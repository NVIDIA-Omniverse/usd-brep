// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmBSplineSurfaceNLib.cpp 
* PURPOSE: Implementation of methods used exclusively with
*    NLib product.  
**********************************************************************/

#include "StdAfx.h"
#include <nurbs.h>
#include <SmBSplineCurve.h>
#include <SmBSplineSurface.h>
#include <SmGraphicsNLib.h>
#include <SmTangentField.h>
#include <SmGeomUtility.h>
#include <SmAssertArray.h>

#include <NL_CrvAdv.h>      /* Advanced NL_CURVE functions      */
#include <NL_SrfAdv.h>      /* Advanced NL_SURFACE functions    */


#ifndef __ATOMIC__
#define __ATOMIC__
#include <atomic>
#endif

#ifdef SM_DEBUG_CODE
 #include <SmEdge.h>
 #include <SmFace.h>
 #include <SmBrep.h>
 #include <SmGap.h>
 #include <SmNurbsCrv.h>
#endif // SM_DEBUG_CODE


// This object automatically calls the initilization and termination of the GW nurb volume stack.
class SmNLibStackHandler 
{
  private:
    NL_STACKS * m_pStacks;
  public:
    SmNLibStackHandler(NL_STACKS * pStacks) { m_pStacks = pStacks; N_InitNurbs(m_pStacks); }
   ~SmNLibStackHandler()                 { N_EndNurbs(m_pStacks); }
} ; // end class SmNLibStackHandler

// MACROS
#define NL_SER(a) { NL_FLAG gw_err = (a); \
                    if (gw_err == NL_YES) { SER(SM_ERR); } \
                  }

#define COPY_XYZ(from,to) (to).x = (from).x; (to).y = (from).y; (to).z = (from).z;

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
static SmStatus sm_BuildNlibCurve
  (SmBSplineCurve *pBSC,
   NL_STACKS & nlStacks,
   NL_CURVE & rCurve)
{
    if ( pBSC == NULL )
    { SER( SM_ERR_INVALID_INPUT ); }

    N_CrvInitArrays(&rCurve);
    NL_CURVE *pCur = pBSC->GetOrCreateGwNurbPointer();
    NL_SER(N_CrvCopy(pCur,&rCurve,&nlStacks));
    return SM_SUCCESS;

} // end sm_BuildNlibCurve

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
static NL_CURVE * sm_CreateNlibCurve
  (SmBSplineCurve *pBSC,
   NL_STACKS & nlStacks)
{
    NL_CURVE *pNewCur = N_AllocCrvAndArrays((NL_INDEX)pBSC->GetNumberControlPoints(),
        (NL_DEGREE)pBSC->GetDegree(), (NL_INDEX)pBSC->GetNumberNaturalKnots(),&nlStacks);

    NL_CURVE *pCur = (NL_CURVE*)pBSC->GetOrCreateGwNurbPointer();
    if (N_CrvCopy(pCur,pNewCur,&nlStacks)) {
        SE(SM_ERR);
        return NULL;
    }
    return pNewCur;

} // end sm_CreateNlibCurve

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
static NL_SURFACE * sm_CreateNlibSurface
  (SmBSplineSurface *pBSS,
   NL_STACKS & nlStacks)
{
    NL_SURFACE *pNewSur = N_AllocSrfAndArrays((NL_INDEX)pBSS->GetNumberControlPoints(SM_SP_U),
                                           (NL_INDEX)pBSS->GetNumberControlPoints(SM_SP_V),
                                           (NL_DEGREE)pBSS->GetDegree(SM_SP_U),
                                           (NL_DEGREE)pBSS->GetDegree(SM_SP_V),
                                           (NL_INDEX)pBSS->GetNumberNaturalKnots(SM_SP_U),
                                           (NL_INDEX)pBSS->GetNumberNaturalKnots(SM_SP_V),&nlStacks);

    NL_SURFACE *pSur = pBSS->GetOrCreateGwNurbPointer();
    if (N_SrfCopy(pSur,pNewSur,&nlStacks)) {
        SE(SM_ERR);
        return NULL;
    }
    return pNewSur;

} // end sm_CreateNlibSurface

/*******************************************************************//**
PURPOSE: Modify the surface so that a given curve becomes a new
    boundary of the surface. 
    
NOTES:       
        That is, the old boundary curve is replaced
    with the new curve. The surface is modified inward a small amount
    (controlled by dPercentAffected) and transitions smoothly into the
    original surface. Any of the four boundaries may be modified.

This method is only available for users with NLib
    
***********************************************************************/
SmStatus SmBSplineSurface::AdjustBoundaryCurve
  (const SmBSplineCurve * cpNewBoundaryCurve,
   SmSurfParamType        eSurfDir,              // u- or v-direction
   SmBoolean              bIsNewBdryAtMin,       // TRUE - u(v)-min FALSE - u(v)-max 
   double                 dPercentAffected)
{
    NL_STACKS nlStacks;
    SmNLibStackHandler sSH(&nlStacks);
    NL_SURFACE nlSurf;
    N_SrfInitArrays(&nlSurf);

    NL_SURFACE * nlSurfP = GetOrCreateGwNurbPointer() ;
    NER(nlSurfP);

    NL_CURVE   * nlCurvP = ((SmBSplineCurve *)cpNewBoundaryCurve)->GetOrCreateGwNurbPointer();

    SmExtent2d sDomain = GetNaturalUVDomain();
    NL_FLAG nlStartOrEnd = NL_START;
    SmPoint2d sUV = sDomain.Evaluate(dPercentAffected,dPercentAffected);
    if (!bIsNewBdryAtMin) {
        // Adjust MAX boundary
        nlStartOrEnd = NL_END;
        sUV = sDomain.Evaluate(1.0-dPercentAffected,1.0-dPercentAffected);
    }
    NL_FLAG nlUorV = 1;
    NL_REAL nlUV = sUV.x; // A parameter value (u or v) which controls how
    // far inward from the affected boundary the surface is modified
    if (eSurfDir == SM_SP_V) {
        nlUorV = 2;
        nlUV = sUV.y;
    }

    NL_SER(N_SrfModifyBoundaryCrv(nlSurfP,nlCurvP,nlUorV,nlStartOrEnd,nlUV, NL_CMAX,&nlSurf,&nlStacks,&nlStacks));

    const SmContext * pContext = GetContext();
    NER(pContext);

    SmBSplineSurface *pTemp = new (*pContext) SmBSplineSurface((gw_SURFACE *)&nlSurf) ;
    SmObjDelete sClean(pTemp);

    // swap NurbPointers
    gw_SURFACE * nlNewSur = pTemp->GetOrCreateGwNurbPointer();
    pTemp->m_pNurb       = nlSurfP ;
    m_pNurb              = nlNewSur;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
    if (bDebugMe) {
        sm_GraphicsLoop();
        smgfx_SetColor(0,0,1);
        DrawUV(1,1);
        Dump();
    }
#endif

    return SM_SUCCESS;

} // end SmBSplineSurface::AdjustBoundaryCurve

/*******************************************************************//**
PURPOSE: This routine approximates free-form offsets of a NURBS surface
     by a process consisting of point sampling, interpolation and knot 
     removal.

NOTES: This method is only available for users of NLib
    
    ParamType = 0 = INHERITED(default)  1 = NL_CHORDLENGTH,  2 = NL_CENTRIPETAL

    Note1: 'Inherited' adds lots of knots, but maybe essential if a uv curve
            refers to the original parameterization. 'Chordlength' is preferable if
            there is just a surface, so that re-parameterisation is OK.
    Note2: Domain is preserved even if 'Chordlength' to match uv curve data

    You can rebuild the UV trimming curves using:
            SmTrimmingTools::ClearUVTrimCurves (pFace);
            SmTrimmingTools::CreateUVTrimCurves (pFace);
            SER(ReplaceSurface(pFace,pNewSurf,FALSE,NULL));

***********************************************************************/
SmStatus SmBSplineSurface::ApproximateOffsetSurface
  (const SmContext   & crContext,       // in : context for new object construction
   double              dOffsetDistance, // in : offset distance along normal (neg for insets)
   double              dApproxTol3d,    // in : max allowed approximation distance
   SmBSplineSurface *& rpNewSurf,       // out: newly constructed approximation surface
   int                 iParamType)      // in : specify approximation test method
                                        //      0 = INHERITED(default)   // iParamType values do not match
                                        //      1 = NL_CHORDLENGTH,         // NLib iParam values
                                        //      2 = NL_CENTRIPETAL
                                        //      3 = NL_UNIFORM
{
  NL_STACKS nlStacks;
  SmNLibStackHandler sSH(&nlStacks);
  NL_SURFACE nlSurf;
  NL_PARAMETER nlULeft = 0.0, nlURight = 0.0, nlVLeft = 0.0, nlVRight = 0.0;
  N_SrfInitArrays(&nlSurf);

  NL_SURFACE * nlSurfP = GetOrCreateGwNurbPointer() ;
  NER(nlSurfP);

  // cubic degree
  NL_DEGREE lnDeg = 3;

  // in case input was from a face, capture the domain
  if (iParamType != 0) 
    { 
      N_SrfGetParameterBounds(nlSurfP, &nlULeft, &nlURight, &nlVLeft, &nlVRight); 
    }

  // map SMLib Param vals to NLib Param vals - yuk
  NL_FLAG nlParam =   (iParamType == 1) ? NL_CHORDLENGTH
                    : (iParamType == 2) ? NL_CENTRIPETAL
                    : (iParamType == 3) ? NL_UNIFORM
                    : NL_INHERITED ;

  // pass the call along to NLib
  NL_SER(N_SrfOffset(nlSurfP,
                     dOffsetDistance,
                     lnDeg,
                     lnDeg,
                     nlParam, 
                     10.0*dApproxTol3d,
                     dApproxTol3d,
                    &nlSurf,
                    &nlStacks)); 

   // reset domain to original ... to approx match trim.curves if face
  if (iParamType != 0)
    {     
      N_SrfReparam(&nlSurf, nlULeft, nlURight, nlVLeft, nlVRight);   
    }

  // use the new Nurb to make a new BSpline
  rpNewSurf = new (crContext) SmBSplineSurface((gw_SURFACE *)&nlSurf) ;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  if (bDebugMe) 
    {
      sm_GraphicsLoop();
      smgfx_SetColor(0,0,1);
      rpNewSurf->DrawUV(1,1);
      rpNewSurf->Dump();
    }
#endif // SM_DEBUG_CODE

  // all done
  return SM_SUCCESS;

} // end SmBSplineSurface::ApproximateOffsetSurface

/*******************************************************************//**
PURPOSE: This routine computes a NURBS surface approximation to a given
    grid-set of (nxm) points.  When computed without a tolerance
    the return surface has (nxm) control points.  When computed with a tolerance
    the returned surface may have a lower control point count.

NOTES: User may specify a tolerance for approximation
    with error bound, otherwise approximate with least-squares, i.e.

       when pdOptTolerance == NULL, call N_FitSrfLstSqApprox(),
       else                         call N_FitSrfApproxTol().
 
    The order of the points in crPoints is expected to be from a sampled
    grid and must be row dominant, e.g. for a 3x4 sample grid

      sGridPoint[0][0] = crPoint[0]
      sGridPoint[0][1] = crPoint[1]
      sGridPoint[0][2] = crPoint[2]
      sGridPoint[0][3] = crPoint[3]

      sGridPoint[1][0] = crPoint[4]
      sGridPoint[1][1] = crPoint[5]

      sGridPoint[2][0] = crPoint[8]
      sGridPoint[2][3] = crPoint[11]

    This method is only available for users with NLib

    bParamOption: NL_YES: use knot vector obtained from knot removal
                       for least-squares approximation (default)
                  NL_NO : compute new knot vector for  least-squares
                       approximation
***********************************************************************/
SmStatus SmBSplineSurface::ApproximatePoints
  (const SmContext           & crContext,      // in : New Object Context
   const SmTArray<SmPoint3d> & crPoints,       // in : Tgt Points arranged in a sample grid sized:[lNumRows x lNumCols] points
                                               //       ordered:P[row][col] = p[row*lNumCols + col]
   ULONG                       lNumRows,       // in : Number of rows in crPoints array
   ULONG                       lNumCols,       // in : Number of cols in crPoints array
   ULONG                       lUDegree,       // in : Output surface degree U
   ULONG                       lVDegree,       // in : Output surface degree V
   double                    * pdOptTolerance, // in : NULL    = use N_FitSrfLstSqApprox() to approximate
                                               //          a (lNumRows)x(lNumCols) controlPoint surface to crPoints
                                               //      NotNULL = use N_FitSrfApproxTol() to approximate
                                               //          a possible smaller controlPoint count surfact to crPoints
                                               //          to within specified tolerance.  Larger tolerances yield
                                               //          smaller control point surfaces.
   SmBSplineSurface         *& rpNewSurf,      // out: Constructed surface
   SmBoolean                   bParamOption)   // in : when using N_FitSrfApproxTol(), as selected by pdOptTolerance value,
                                               //       specify method used to reduce knot counts.
                                               //       TRUE  = knots computed by knot reduction of last iteration surface
                                               //       FALSE = knots computed by clumping sample point param values
{
  // check input
  SM_ASSERT(   lUDegree < lNumRows
            && lVDegree < lNumCols);

  // NLib needed locals
  NL_SURFACE nlSurf;
  NL_STACKS  nlStacks;
  SmNLibStackHandler sSH(&nlStacks);
  N_SrfInitArrays(&nlSurf);

  // set N_FitSrfLstSqApprox() input arguments
  NL_INDEX  nlNRows = lNumRows-1;
  NL_INDEX  nlNCols = lNumCols-1;
  NL_POINT **nlPts = N_AllocPt2dArray(nlNRows,nlNCols,&nlStacks);

  ULONG lIndex =0;
  ULONG ii, jj;
  for (ii=0; ii<lNumRows; ii++)
    {
      for (jj=0; jj<lNumCols; jj++)
        {
          SmPoint3d sPnt = crPoints[lIndex++];
          COPY_XYZ( sPnt, nlPts[ii][jj] );
        } // end iter every col
    } // end iter every row

  NL_DEGREE nlDegU = (NL_DEGREE)lUDegree;
  NL_DEGREE nlDegV = (NL_DEGREE)lVDegree;

  if (pdOptTolerance == NULL)
    {
      // Least-squares NURBS approximation for a sampled-grid of points.
      //  build a (nlNPtsU+1) x (nlNPtsV+1) control point, and
      //          (nlNPtsU+nlDegU+1) x (nlNPtsV+nlDegV+1) knot surface which attempts to place
      //          all nlPts row and col of points on isoparameter curves.
      //  works well well when the nlPts grid points are regularly sampled.
      NL_INDEX nlNPtsU = nlNRows;
      NL_INDEX nlNPtsV = nlNCols;
      NL_SER(N_FitSrfLstSqApprox(nlPts,            // Point set
                                 nlNRows,nlNCols,  //   sized:[nlNRows+1][nlNCols+1]
                                 nlNPtsU,nlNPtsV,  // Output surface ControlPoint counts
                                 nlDegU,nlDegV,    // Output surface degrees
                                 NL_CHORDLENGTH,   // set uv param values from Chord length parameterization
                                 &nlSurf,&nlStacks)); // the output surface and its memory stack
    }
  else
    {
      // Approximation with error bound.
      //   use an iterative method to get closer and closer approximations to the points.
      //   the output control point count is increased until the approximation gets within tolerance.
      // note: if you get wavy results, set bParamOption to FALSE
      NL_SER(N_FitSrfApproxTol(nlPts,               // Point set
                               nlNRows,nlNCols,     //   sized:[nlNRows+1][nlNCols+1]
                               2,nlDegU,            // Iteration Start and Final Degree U values
                               2,nlDegV,            // Iteration Start and Final Degree V values
                               *pdOptTolerance,     // error tolerance
                               NL_SINGLE,           // use single knots for approximation
                               (NL_FLAG)bParamOption, // internal method of updating knot vectors within each iteration.
                               &nlSurf,&nlStacks)); // the output surface and its memory stack
    }

  // set output
  rpNewSurf = new (crContext) SmBSplineSurface((gw_SURFACE *)&nlSurf) ;  // gwc: was TRUE = run AssertValidAndHeal on New BSplineSurface

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  if (bDebugMe)
    {
      ULONG di ;
      rpNewSurf->Dump();
      crPoints.Dump() ;

      sm_GraphicsLoop();
      smgfx_SetLook(1,2, 0,0,1); rpNewSurf->DrawUV(1,1); sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 1,0,0); for(di=0;di<crPoints.GetSize();di++) crPoints[di].Draw() ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif

  // all done
  return SM_SUCCESS;

} // end SmBSplineSurface::ApproximatePoints

/*******************************************************************//**
PURPOSE: This routine computes a NURBS bicubic surface approximation 
    to a given array of (nxm) points and normals, the output surface
    has about 9 times as many control points as sample points. 

NOTES: 
    This is intended for sparse data, since there are approx the same 
    number of spans in the output surface as there were points.
    The routine computes a non-rational bicubic with multiple interior knots.

    This method is only available for users with NLib
***********************************************************************/
SmStatus SmBSplineSurface::ApproximatePointNormals
  (const SmContext            & crContext,  // in : new object context
   const SmTArray<SmPoint3d>  & crPoints,   // in : Grid of Sample Point Positions, sized:[lNumRows][lNumCols]
                                            //       ordered:P[row][col] = p[row*lNumCols + col]
   const SmTArray<SmVector3d> & crNormals,  // in : Grid of Sample Point Normals, sized:[lNumRows][lNumCols]
                                            //       ordered:P[row][col] = p[row*lNumCols + col]
   ULONG                        lNumRows,   // in : number of rows in Sample Point grid
   ULONG                        lNumCols,   // in : number of cols in Sample Point grid
   SmBSplineSurface          *& rpNewSurf)  // out: approximating surface
{
  // NLib locals
  NL_STACKS nlStacks;
  SmNLibStackHandler sSH(&nlStacks);
  NL_SURFACE nlSurf;
  N_SrfInitArrays(&nlSurf);

  // N_FitPtsNormals() arguments
  NL_INDEX        nlNRows = lNumRows-1;
  NL_INDEX        nlNCols = lNumCols-1;
  NL_POINT   **nlPts = N_AllocPt2dArray(nlNRows,nlNCols,&nlStacks);
  NL_VECTOR     **nlNorms = N_AllocPt2dArray(nlNRows,nlNCols,&nlStacks);

  // copy SMLib arrays into NLib arrays
  ULONG lIndex=0, ii, jj;
  for (ii=0; ii<lNumRows; ii++) 
    {
      for (jj=0; jj<lNumCols; jj++) 
        {
          SmPoint3d sPnt = crPoints[lIndex];
          COPY_XYZ(sPnt,nlPts[ii][jj]);
          SmVector3d sNrm = crNormals[lIndex++];
          COPY_XYZ(sNrm,nlNorms[ii][jj]);
        } // end iter every col
    } // end iter every row
 
  //
  NL_SER(N_FitPtsNormals(nlPts, nlNorms, nlNRows, nlNCols, NL_AKIMA, &nlSurf, &nlStacks));

  // set output
  rpNewSurf = new (crContext) SmBSplineSurface((gw_SURFACE *)&nlSurf) ; // gwc: was TRUE = run AssertValidAndHeal on New BSplineSurface

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  if (bDebugMe) 
    {
      ULONG di ;
      rpNewSurf->Dump();
      crPoints.Dump() ;
      crNormals.Dump() ;

      sm_GraphicsLoop();
      smgfx_SetLook(1,2, 0,0,1); rpNewSurf->DrawUV(1,1); sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 0,1,0); for(di=0;di<crPoints.GetSize();di++) crPoints[di].Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 1,0,0); for(di=0;di<crNormals.GetSize();di++) { SmVector3d pt = crPoints[di]; crNormals[di].Draw(&pt); } sm_GraphicsLoop();
      sm_GraphicsLoop() ;
    }
#endif

  // all done
  return SM_SUCCESS;

} // end SmBSplineSurface::ApproximatePointNormals

/*******************************************************************//**
PURPOSE: This routine computes a least squares b-spline surface 
    approximation to a random (non-NxM) set of points. i.e. point cloud
    and an optional set of natural boundary position and cross derivative curves. 

NOTES:
    Optionally, users may input (non-rational) boundary curves as constraints.
    If the boundary curves are given, the flag indicating the complexity of
    the surface should also be given. The output surface controlPoint counts
    in the U and V directions will be at least as large as the largest 
    appropriate control point count boundary curve.
    
Example:
    (A) The 'lComplexity' option (when boundary curves are specified)
        tells the routine how complicated the surface might be.
        The possible values are:
          1: Gives good results for simple patches. It is stable and reliable,
             however, tends to miss details inherent in the data.
          2: Gives good results in general, The surface may be
             smoother than level 3, however, it is also the slowest. 
          3: Fast to compute and is able to capture internal details,
             however, it tends to wrinkle the surface especially if the data
             represents a bit more complex patch.

    (B) The specified lMaxUCtrlPnts & lMaxVCtrlPnts can make great difference
        in the results. Users are suggested to choose numbers that may best 
        'describe' the size & shape of the data points and follow the criteria:

            (lMaxUCtrlPnts+1) * (lMaxVCtrlPnts+1) <= Total # of Data Points.

        For example, suppose we have 100 sample points, then choose 8x8 or 7x7
        if a 'square-like' shape of surface is approximated. Other choices such as:
        9x7, 10x5,...etc. can be used to describe a 'rectangle-like' surfaces.
        
    This method is only available for users with NLib
***********************************************************************/
SmStatus SmBSplineSurface::ApproximateRandomPoints
  (const SmContext                 & crContext,       // in : New Object Context
   const SmTArray<SmPoint3d>       & crPoints,        // in : target Points
   ULONG                             lMaxUCtrlPnts,   // in : Output surface controlPoints U maximum count 
   ULONG                             lMaxVCtrlPnts,   // in : Output surface controlPoints V maximum count 
   ULONG                             lUDegree,        // in : Output surface degree U
   ULONG                             lVDegree,        // in : Output surface degree V
   const SmTArray<SmBSplineCurve*> * cpOptBdryCurves, // in : The first two will be U-boundaries (v-min & v-max) and
                                                      //      the last  two will be V-boundaries (u-min & u-max)
                                                      //      Will increase lMaxUCtrlPnts and lMaxVCtrlPnts values if needed.
   ULONG                             lComplexity,     // in : 1 for simple patches (cheap)
                                                      //      2 when internal details are required (expensive), default:[2]
                                                      //      3 for more points and smoother surfaces (inbetween)
   SmBSplineSurface               *& rpNewSurf)       // out: approximated surface
{
  // check input
  SM_ASSERT(crPoints.GetSize() > 0);
  SM_ASSERT(cpOptBdryCurves == NULL || cpOptBdryCurves->GetSize() == 4) ;
  SM_ASSERT(cpOptBdryCurves == NULL || lComplexity == 1
                                    || lComplexity == 2
                                    || lComplexity == 3) ;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  // draw and dump inputs
  if(bDebugMe)
    {
      SmEdge *pEdge = NULL ;
      SmBrep *pBrep = NULL ;
      if(cpOptBdryCurves) { for(ULONG ii=0;ii<4;ii++)
                             { SM_DUMP_AND_ASSERT_VALID(cpOptBdryCurves->GetAt(ii)) ;
                               if(cpOptBdryCurves->GetAt(ii)->GetEdge())
                                 { pEdge = (SmEdge *)(cpOptBdryCurves->GetAt(ii)->GetEdge()) ;
                                   if(pEdge->GetBrep()) {pBrep = pEdge->GetBrep() ; }
                          }  }   }
      smgfx_Erase() ;
      if(cpOptBdryCurves) 
        { smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
          smgfx_SetLook(2,3, 1,0,1) ; if(pEdge) pEdge->Draw() ; sm_GraphicsLoop() ;
          smgfx_SetLook(3,4, 0,0,1) ; if(cpOptBdryCurves->GetAt(0)) cpOptBdryCurves->GetAt(0)->Draw() ; sm_GraphicsLoop() ;
          smgfx_SetLook(3,4, 0,1,1) ; if(cpOptBdryCurves->GetAt(1)) cpOptBdryCurves->GetAt(1)->Draw() ; sm_GraphicsLoop() ;
          smgfx_SetLook(3,4, 0,1,0) ; if(cpOptBdryCurves->GetAt(2)) cpOptBdryCurves->GetAt(2)->Draw() ; sm_GraphicsLoop() ;
          smgfx_SetLook(3,4, 1,0,0) ; if(cpOptBdryCurves->GetAt(3)) cpOptBdryCurves->GetAt(3)->Draw() ; sm_GraphicsLoop() ;
        }
      smgfx_SetLook(4,5,.5,.8,1); for(ULONG ii=0;ii<crPoints.GetSize();ii++) { crPoints[ii].Draw() ; } sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // NLib locals
  NL_STACKS nlStacks;
  SmNLibStackHandler sSH(&nlStacks);
  NL_SURFACE nlSurf;
  N_SrfInitArrays(&nlSurf);

  // copy input points into NLib P array
  ULONG i;
  NL_INDEX     np = crPoints.GetSize()-1;
  NL_POINT *P  = N_AllocPt1dArray(np,&nlStacks);
  for (i=0; i<crPoints.GetSize(); i++) 
    {
      SmPoint3d sPnt = crPoints[i];
      COPY_XYZ(sPnt,P[i]);
    }
  
  // copy input curves (if any) into curs array
  NL_CURVE  *curs[4];
  NL_CURVE  **bdys = NULL;
  NL_REAL   *u, *v;
  NL_POINT   *T;
  NL_INDEX  nt;

  // when optional boundary curves are given
  if (cpOptBdryCurves) 
    {
      // Boundary curves are given
      if (cpOptBdryCurves->GetSize() != 4) SER(SM_ERR);
      for (ULONG ii=0; ii<4; ii++) 
        {
          SmBSplineCurve * pCurve = (*cpOptBdryCurves)[ii];
          if (pCurve->IsRational()) 
            {
              SER(SM_ERR); // Only non-rational is allowed 
            }
          curs[ii] = sm_CreateNlibCurve(pCurve,nlStacks);
        }

      // Make curves 1 & 2 compatible 
      NL_SER(N_CrvsMakeCompatible(curs,1,&nlStacks)); 

      // Make curves 3 & 4 compatible
      NL_SER(N_CrvsMakeCompatible(&curs[2],1,&nlStacks));


#ifdef SM_DEBUG_CODE
      if(bDebugMe)
        {
          // These are commented out for now
          // Unfortunately, adding this function as an SmBSplineCurve method
          // causes unexplained issues in the merge operator
          //SmBSplineCurve::Dump_NCrv(curs[0]) ;
          //SmBSplineCurve::Dump_NCrv(curs[1]) ;

          //SmBSplineCurve::Dump_NCrv(curs[2]) ;
          //SmBSplineCurve::Dump_NCrv(curs[3]) ;
        }
#endif // SM_DEBUG_CODE

      // Derive u,v parameters
      NL_REAL  tol = 0.01; // 1%
      NL_REAL  per = 0.1;  // 10%    // gwc: this choice can cause adequate surface sampling to become subsampled.
      NL_FLAG  thf = NL_YES ;     /* oneof NL_YES= thin data set; tol is for deomposition, thinning, and projection */
                                  /*       NL_NO = don't thin data; tol is for decomposition and projection */

      NL_FLAG bsf;
      switch (lComplexity) 
        {
          case 1: // For simple patches 
                  bsf = NL_BILINEARCOONS;
                  break;
          case 3: // Internal details are required
                  bsf = NL_BICUBICCOONS;
                  break;
          case 2: // For more points & smoother surface (slowest)
          default:
                  bsf = NL_BIQUADRATIC;
                  break;
        }
    
      // Get uv points for a subset of control points.
      //   Use the boundary curves and a percentage of the P sample points to build a 
      //   Bilinear Coons, Bicubic Coons, or Biquadratic base surface. The P points 
      //   are then projected to the Base surface to get u,v guesses for each point.
      NL_SER(N_FitCalcSrfParamsBoundarySrf
                   (P,        /* in : target points, sized:[n+1] */                                                          
                    np,       /* in : highest index in P */                                                                  
                    curs,     /* in : compatible U boundaries, nlCurU[0] = vMin, nlCurU[1] = vMax */                             
                    &curs[2], /* in : compatible V boundaries, nlCurV[0] = uMin, nlCurV[1] = uMax */                             
                    thf,      /* in : oneof NL_YES= thin data set; tol is for deomposition, thinning, and projection */      
                              /*            NL_NO = don't thin data; tol is for decomposition and projection */              
                    bsf,      /* in : oneof NL_BILINEARCOONS=  base surface is a bilinearly blended Coons patch  */          
                              /*            NL_BIQUADRATIC  =  base surface is a biquadratic surface patch       */          
                              /*            NL_BICUBICCOONS =  base surface is a bicubically blended Coons patch */          
                    tol,      /* in : Surface flatness tolerance;  MUST BE A RELATIVE TOLERANCE!! */                      
                              /*            1%, I.E. 0.01 IS SUGGESTED.                              */                      
                    per,      /* in : not used - percentage of points selected to form base surface - suggest 10% = 0.10 */             
                    -1,       /* in : highest ControlPoint U index in base surface - neg. number = compute internally */     
                    -1,       /* in : highest ControlPoint V index in base surface - neg. number = compute internally */     
                    NULL,     /* in : Opt Base Surface, NULL to ignore */                                                    
                    &T,       /* out: Thinned points of P successfully projected onto base surface, sized:[m+1] */           
                    &u,       /* out: U parameters for points in T, sized:[m+1] */                                           
                    &v,       /* out: V parameters for points in T, sized:[m+1] */                                           
                    &nt,      /* out: size parameter, number of values in T, u, and v */                                                                      
                    &nlStacks));    /* in : output memory stack */                                                                 
                                                                                                                             
#ifdef SM_DEBUG_CODE
      // draw and dump inputs with subsample point array T
      if(bDebugMe)
        {
          ULONG ii ;
          SmEdge *pEdge = NULL ;
          SmBrep *pBrep = NULL ;
          if(cpOptBdryCurves) { for(ii=0;ii<4;ii++)
                                 { if(cpOptBdryCurves->GetAt(ii)->GetOwner())
                                     { pEdge = (SmEdge *)(cpOptBdryCurves->GetAt(ii)->GetEdge()) ;
                                       if(pEdge->GetBrep()) {pBrep = pEdge->GetBrep() ; }
                              }  }   }
          smgfx_Erase() ;
          if(cpOptBdryCurves) 
            { smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
              smgfx_SetLook(2,3, 1,0,1) ; if(pEdge) pEdge->Draw() ; sm_GraphicsLoop() ;
              smgfx_SetLook(3,4, 0,0,1) ; if(cpOptBdryCurves->GetAt(0)) cpOptBdryCurves->GetAt(0)->Draw() ; sm_GraphicsLoop() ;
              smgfx_SetLook(3,4, 0,1,1) ; if(cpOptBdryCurves->GetAt(1)) cpOptBdryCurves->GetAt(1)->Draw() ; sm_GraphicsLoop() ;
              smgfx_SetLook(3,4, 0,1,0) ; if(cpOptBdryCurves->GetAt(2)) cpOptBdryCurves->GetAt(2)->Draw() ; sm_GraphicsLoop() ;
              smgfx_SetLook(3,4, 1,0,0) ; if(cpOptBdryCurves->GetAt(3)) cpOptBdryCurves->GetAt(3)->Draw() ; sm_GraphicsLoop() ;
            }
          smgfx_SetLook(2,3,.5,.8,1); for(ii=0;ii<crPoints.GetSize();ii++) { crPoints[ii].Draw() ; } sm_GraphicsLoop() ;
          smgfx_SetLook(5,6,1,.6,.5); for(ii=0;ii<=(ULONG)nt;ii++) { ((SmPoint3d&)(T[ii])).Draw() ; } sm_GraphicsLoop() ;
          sm_GraphicsLoop() ;
        }
#endif // SM_DEBUG_CODE

      // Re-arrange the curve array in the following order:
      // i=0=BOTTOM, i=1=RIGHT, i=2=TOP, i=3=LEFT  from  { Bot Top Lft Rgt }
      NL_CURVE *tmp = curs[3];
      curs[3] = curs[2];
      curs[2] = curs[1];
      curs[1] = tmp;
      bdys = curs;
    } // end optional boundary curves given branch
  else // optional boundary curves not given branch
    {
      // get uv Points for each P point by projecting to a plane.
      //   this works ok when the sample points come from a pretty planar surface.
      u = N_AllocReal1dArray(np,&nlStacks);
      v = N_AllocReal1dArray(np,&nlStacks);
      SmPoint3d sPnt(0.0,0.0,0.0);
      NL_POINT    Og;
      NL_VECTOR  X, Y, Z;
      COPY_XYZ(sPnt,Og);
      COPY_XYZ(sPnt,X);
      COPY_XYZ(sPnt,Y);
      COPY_XYZ(sPnt,Z);
      NL_PARAMETER  us = 0.0, ue = 1.0; // u-bounds
      NL_PARAMETER  vs = 0.0, ve = 1.0; // v-bounds

      // get uv points for each P point by projecting to a plane fitted to the sample points
      NL_SER(N_FitSrfCalcParams(P,np,             // Sample Points, sized:[np+1]
                               NL_NO,             // NL_NO = compute projection plane
                               Og,X,Y,Z,          // not used when computing projection plane
                               us,ue,vs,ve,       // output parameter range
                               u,v));             // output uv points for each P point
      T = P;
      nt = np;

    } // end map P to uv points by projecting to a plane branch

  // output surface U and V dir control point counts must be as large as U and V dir boundary control point counts
  // gwc:Bug151 - modified n,m computation to equal the max boundary curve control point count
  if(cpOptBdryCurves)
    {
      ULONG lBdryCPtCnt0 = (cpOptBdryCurves->GetAt(0) != NULL) ? cpOptBdryCurves->GetAt(0)->GetNumberControlPoints() : lMaxUCtrlPnts ;
      ULONG lBdryCPtCnt1 = (cpOptBdryCurves->GetAt(1) != NULL) ? cpOptBdryCurves->GetAt(1)->GetNumberControlPoints() : lMaxUCtrlPnts ;
      ULONG lBdryCPtCnt2 = (cpOptBdryCurves->GetAt(2) != NULL) ? cpOptBdryCurves->GetAt(2)->GetNumberControlPoints() : lMaxVCtrlPnts ;
      ULONG lBdryCPtCnt3 = (cpOptBdryCurves->GetAt(3) != NULL) ? cpOptBdryCurves->GetAt(3)->GetNumberControlPoints() : lMaxVCtrlPnts ;
      lMaxUCtrlPnts = smos_3Max(lMaxUCtrlPnts, lBdryCPtCnt0, lBdryCPtCnt1) ;
      lMaxVCtrlPnts = smos_3Max(lMaxVCtrlPnts, lBdryCPtCnt2, lBdryCPtCnt3) ;
    }

  // locals 
  NL_INDEX  n = lMaxUCtrlPnts-1;
  NL_INDEX  m = lMaxVCtrlPnts-1;
  NL_DEGREE p = (NL_DEGREE)lUDegree;
  NL_DEGREE q = (NL_DEGREE)lVDegree;
  NL_REAL  *wts = NULL;
  NL_CURVE **ders = NULL;

  // Fit the sample points P, given a uv Point for each P point and optional boundary curves
  NL_SER(N_FitSrfLstSqBoundary
    (T,     /* in : sample points, sized:[np+1] */                                                                   
     wts,   /* in : opt least squares weights, sized:[np+1], */                                                      
            /*      NULL to ignore, sized:[np+1] */                                                                  
     u,     /* in : u param for every P point, sized:[np+1] */                                                       
     v,     /* in : v param for every P point, sized:[np+1] */                                                       
     nt,    /* in : size param, size of T, u, and v arrays */                                                                                    
     bdys,  /* in : opt compatible bdry curves: 0=Bottom, 1=Right, 2=Top, 3=Left,  */                                
            /*      bdys[i] = NULL to ignore one or more curves, */                                                  
            /*      bdys = NULL to ignore all curves */                                                              
     ders,  /* in : opt associated cross-derivative curves, sized:[4], NULL to ignore.           */                  
            /*      Curves must be compatible and match bdys[i] derivative values in the corners */                  
     n,     /* in : output sur highest ControlPoint index U */                                                       
     m,     /* in : output sur highest ControlPoint index V */                                                       
     p,     /* in : output sur degree U */                                                                           
     q,     /* in : output sur degree V */                                                                           
     NULL,  /* in : opt sur KnotVector U, NULL=function computes a likely knot vector */                             
     NULL,  /* in : opt sur KnotVector V, NULL=function computes a likely knot vector */                             
     NL_FULL,  /* in : memory flag: NL_FULL   = more memory, faster, */                                                 
            /*                   NL_SPARSE = less memory, slower  */                                                 
     NL_SVD,   /* in : solver flag: NL_SVD=SingleValueDecomposition, */                                                   
            /*                   NL_LUPIV=LU Decomposition with partial pivoting */                                  
     NL_YES,   /* in : solver cntrl: NL_YES=use the aflg internal solver,        */                                     
            /*                    NL_NO=use the user supplied callback solver */                                     
     NULL,  /* in : opt solver callback, use this if you own a better solver, */                                     
            /*        only used when iflg = NL_NO */                                                                 
            /*                                    */                                                                 
     &nlSurf,  /* out: the approximated surface */                                                                      
     &nlStacks)); /* in : sur's memory stack */                                                                            

  rpNewSurf = new (crContext) SmBSplineSurface((gw_SURFACE *)&nlSurf) ;  // gwc: was TRUE = run AssertValidAndHeal on New BSplineSurface
#ifdef SM_DEBUG_CODE
  if (bDebugMe) 
    {
      // dump and draw newSurf with input args
      rpNewSurf->Dump();
      ULONG ii ;
      SmEdge *pEdge = NULL ;
      SmBrep *pBrep = NULL ;
      if(cpOptBdryCurves) { for(ii=0;ii<4;ii++)
                             { if(cpOptBdryCurves->GetAt(ii)->GetEdge())
                                 { pEdge = (SmEdge *)(cpOptBdryCurves->GetAt(ii)->GetEdge()) ;
                                   if(pEdge->GetBrep()) {pBrep = pEdge->GetBrep() ; }
                          }  }   }

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(2,3, 1,0,1) ; if(pEdge) pEdge->Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,0) ; if(rpNewSurf) rpNewSurf->DrawUV(); sm_GraphicsLoop();

      if(cpOptBdryCurves) 
        { smgfx_SetLook(3,4, 0,0,1) ; if(cpOptBdryCurves->GetAt(0)) cpOptBdryCurves->GetAt(0)->Draw() ; sm_GraphicsLoop() ;
          smgfx_SetLook(3,4, 0,1,1) ; if(cpOptBdryCurves->GetAt(1)) cpOptBdryCurves->GetAt(1)->Draw() ; sm_GraphicsLoop() ;
          smgfx_SetLook(3,4, 0,1,0) ; if(cpOptBdryCurves->GetAt(2)) cpOptBdryCurves->GetAt(2)->Draw() ; sm_GraphicsLoop() ;
          smgfx_SetLook(3,4, 1,0,0) ; if(cpOptBdryCurves->GetAt(3)) cpOptBdryCurves->GetAt(3)->Draw() ; sm_GraphicsLoop() ;
        }
      smgfx_SetLook(3,4,.5,.8,1); for(ii=0;ii<crPoints.GetSize();ii++) { crPoints[ii].Draw() ; } sm_GraphicsLoop() ;
      smgfx_SetLook(6,7,1,.6,.5); for(ii=0;ii<=(ULONG)nt;ii++) { ((SmPoint3d&)(T[ii])).Draw() ; } sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // all done
  return SM_SUCCESS;

} // end SmBSplineSurface::ApproximateRandomPoints

/*******************************************************************//**
PURPOSE: This routine computes a least squares b-spline surface 
    approximation to a random (non-NxM) set of points. i.e. point cloud
    and an optional set of natural boundary position and cross derivative curves. 

NOTES:
    Optionally, users may input (non-rational) boundary curves as constraints.
    If the boundary curves are given, the flag indicating the complexity of
    the surface should also be given. The output surface controlPoint counts
    in the U and V directions will be at least as large as the largest 
    appropriate control point count boundary curve.
    
Example:
    (A) The 'lComplexity' option (when boundary curves are specified)
        tells the routine how complicated the surface might be.
        The possible values are:
          1: Gives good results for simple patches. It is stable and reliable,
             however, tends to miss details inherent in the data.
          2: Gives good results in general, The surface may be
             smoother than level 3, however, it is also the slowest. 
          3: Fast to compute and is able to capture internal details,
             however, it tends to wrinkle the surface especially if the data
             represents a bit more complex patch.

    (B) The specified lMaxUCtrlPnts & lMaxVCtrlPnts can make great difference
        in the results. Users are suggested to choose numbers that may best 
        'describe' the size & shape of the data points and follow the criteria:

            (lMaxUCtrlPnts+1) * (lMaxVCtrlPnts+1) <= Total # of Data Points.

        For example, suppose we have 100 sample points, then choose 8x8 or 7x7
        if a 'square-like' shape of surface is approximated. Other choices such as:
        9x7, 10x5,...etc. can be used to describe a 'rectangle-like' surfaces.
        
    This method is only available for users with NLib
***********************************************************************/
SmStatus SmBSplineSurface::ApproximateRandomPointsDeformable
  (const SmContext                 & crContext,       // in : New Object Context
   const SmTArray<SmPoint3d>       & crPoints,        // in : target Points
   ULONG                             lMaxUCtrlPnts,   // in : Output surface controlPoints U maximum count 
   ULONG                             lMaxVCtrlPnts,   // in : Output surface controlPoints V maximum count 
   ULONG                             lUDegree,        // in : Output surface degree U
   ULONG                             lVDegree,        // in : Output surface degree V
   const SmTArray<SmBSplineCurve*> * cpOptBdryCurves, // in : The first two will be U-boundaries (v-min & v-max) and
                                                      //      the last  two will be V-boundaries (u-min & u-max)
                                                      //      Will increase lMaxUCtrlPnts and lMaxVCtrlPnts values if needed.
   SmBoolean bPeriodicU,                              // in : If TRUE, v-min and v-max must be the same curve
   SmBoolean bPeriodicV,                              // in : If TRUE, u-min and u-max must be the same curve
   ULONG                             lComplexity,     // in : 1 for simple patches (cheap)
                                                      //      2 when internal details are required (expensive), default:[2]
                                                      //      3 for more points and smoother surfaces (inbetween)
   double dAlpha,                                     // in : Resistance to stretch weight term
   double dBeta,                                      // in : Resistance to bending weight term
   double dStiffness,                                 // in : Scales stiffness matrix. Higher value lessens impact of alpha and beta
   SmBSplineSurface               *& rpNewSurf)       // out: approximated surface
{
  // check input
  SM_ASSERT(crPoints.GetSize() > 0);
  SM_ASSERT(cpOptBdryCurves == NULL || cpOptBdryCurves->GetSize() == 4) ;
  SM_ASSERT(cpOptBdryCurves == NULL || lComplexity == 1
                                    || lComplexity == 2
                                    || lComplexity == 3) ;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  // draw and dump inputs
  if(bDebugMe)
    {
      SmEdge *pEdge = NULL ;
      SmBrep *pBrep = NULL ;
      if(cpOptBdryCurves) { for(ULONG ii=0;ii<4;ii++)
                             { SM_DUMP_AND_ASSERT_VALID(cpOptBdryCurves->GetAt(ii)) ;
                               if(cpOptBdryCurves->GetAt(ii)->GetEdge())
                                 { pEdge = (SmEdge *)(cpOptBdryCurves->GetAt(ii)->GetEdge()) ;
                                   if(pEdge->GetBrep()) {pBrep = pEdge->GetBrep() ; }
                          }  }   }
      smgfx_Erase() ;
      if(cpOptBdryCurves) 
        { smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
          smgfx_SetLook(2,3, 1,0,1) ; if(pEdge) pEdge->Draw() ; sm_GraphicsLoop() ;
          smgfx_SetLook(3,4, 0,0,1) ; if(cpOptBdryCurves->GetAt(0)) cpOptBdryCurves->GetAt(0)->Draw() ; sm_GraphicsLoop() ;
          smgfx_SetLook(3,4, 0,1,1) ; if(cpOptBdryCurves->GetAt(1)) cpOptBdryCurves->GetAt(1)->Draw() ; sm_GraphicsLoop() ;
          smgfx_SetLook(3,4, 0,1,0) ; if(cpOptBdryCurves->GetAt(2)) cpOptBdryCurves->GetAt(2)->Draw() ; sm_GraphicsLoop() ;
          smgfx_SetLook(3,4, 1,0,0) ; if(cpOptBdryCurves->GetAt(3)) cpOptBdryCurves->GetAt(3)->Draw() ; sm_GraphicsLoop() ;
        }
      smgfx_SetLook(4,5,.5,.8,1); for(ULONG ii=0;ii<crPoints.GetSize();ii++) { crPoints[ii].Draw() ; } sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // NLib locals
  NL_STACKS nlStacks;
  SmNLibStackHandler sSH(&nlStacks);
  NL_SURFACE nlSurf;
  N_SrfInitArrays(&nlSurf);

  // copy input points into NLib P array
  ULONG i;
  NL_INDEX     np = crPoints.GetSize()-1;
  NL_POINT *P  = N_AllocPt1dArray(np,&nlStacks);
  for (i=0; i<crPoints.GetSize(); i++) 
    {
      SmPoint3d sPnt = crPoints[i];
      COPY_XYZ(sPnt,P[i]);
    }
  
  // copy input curves (if any) into curs array
  NL_CURVE  *curs[4];
  NL_CURVE  **bdys = NULL;
  NL_REAL   *u, *v;
  NL_POINT   *T;
  NL_INDEX  nt;

  // when optional boundary curves are given
  if (cpOptBdryCurves) 
    {
      // Boundary curves are given
      if (cpOptBdryCurves->GetSize() != 4) SER(SM_ERR);
      for (ULONG ii=0; ii<4; ii++) 
        {
          SmBSplineCurve * pCurve = (*cpOptBdryCurves)[ii];
          if (pCurve->IsRational()) 
            {
              SER(SM_ERR); // Only non-rational is allowed 
            }
          curs[ii] = sm_CreateNlibCurve(pCurve,nlStacks);
        }

      // Make curves 1 & 2 compatible 
      NL_SER(N_CrvsMakeCompatible(curs,1,&nlStacks)); 

      // Make curves 3 & 4 compatible
      NL_SER(N_CrvsMakeCompatible(&curs[2],1,&nlStacks));


#ifdef SM_DEBUG_CODE
      if(bDebugMe)
        {
          // These are commented out for now
          // Unfortunately, adding this function as an SmBSplineCurve method
          // causes unexplained issues in the merge operator
          //SmBSplineCurve::Dump_NCrv(curs[0]) ;
          //SmBSplineCurve::Dump_NCrv(curs[1]) ;

          //SmBSplineCurve::Dump_NCrv(curs[2]) ;
          //SmBSplineCurve::Dump_NCrv(curs[3]) ;
        }
#endif // SM_DEBUG_CODE

      // Derive u,v parameters
      NL_REAL  tol = 0.01; // 1%
      NL_REAL  per = 0.1;  // 10%    // gwc: this choice can cause adequate surface sampling to become subsampled.
      NL_FLAG  thf = NL_YES ;     /* oneof NL_YES= thin data set; tol is for deomposition, thinning, and projection */
                                  /*       NL_NO = don't thin data; tol is for decomposition and projection */

      NL_FLAG bsf;
      switch (lComplexity) 
        {
          case 1: // For simple patches 
                  bsf = NL_BILINEARCOONS;
                  break;
          case 3: // Internal details are required
                  bsf = NL_BICUBICCOONS;
                  break;
          case 2: // For more points & smoother surface (slowest)
          default:
                  bsf = NL_BIQUADRATIC;
                  break;
        }
    
      // Get uv points for a subset of control points.
      //   Use the boundary curves and a percentage of the P sample points to build a 
      //   Bilinear Coons, Bicubic Coons, or Biquadratic base surface. The P points 
      //   are then projected to the Base surface to get u,v guesses for each point.
      NL_SER(N_FitCalcSrfParamsBoundarySrf
                   (P,        /* in : target points, sized:[n+1] */                                                          
                    np,       /* in : highest index in P */                                                                  
                    curs,     /* in : compatible U boundaries, nlCurU[0] = vMin, nlCurU[1] = vMax */                             
                    &curs[2], /* in : compatible V boundaries, nlCurV[0] = uMin, nlCurV[1] = uMax */                             
                    thf,      /* in : oneof NL_YES= thin data set; tol is for deomposition, thinning, and projection */      
                              /*            NL_NO = don't thin data; tol is for decomposition and projection */              
                    bsf,      /* in : oneof NL_BILINEARCOONS=  base surface is a bilinearly blended Coons patch  */          
                              /*            NL_BIQUADRATIC  =  base surface is a biquadratic surface patch       */          
                              /*            NL_BICUBICCOONS =  base surface is a bicubically blended Coons patch */          
                    tol,      /* in : Surface flatness tolerance;  MUST BE A RELATIVE TOLERANCE!! */                      
                              /*            1%, I.E. 0.01 IS SUGGESTED.                              */                      
                    per,      /* in : not used - percentage of points selected to form base surface - suggest 10% = 0.10 */             
                    -1,       /* in : highest ControlPoint U index in base surface - neg. number = compute internally */     
                    -1,       /* in : highest ControlPoint V index in base surface - neg. number = compute internally */     
                    NULL,     /* in : Opt Base Surface, NULL to ignore */                                                    
                    &T,       /* out: Thinned points of P successfully projected onto base surface, sized:[m+1] */           
                    &u,       /* out: U parameters for points in T, sized:[m+1] */                                           
                    &v,       /* out: V parameters for points in T, sized:[m+1] */                                           
                    &nt,      /* out: size parameter, number of values in T, u, and v */                                                                      
                    &nlStacks));    /* in : output memory stack */                                                                 
                                                                                                                             
#ifdef SM_DEBUG_CODE
      // draw and dump inputs with subsample point array T
      if(bDebugMe)
        {
          ULONG ii ;
          SmEdge *pEdge = NULL ;
          SmBrep *pBrep = NULL ;
          if(cpOptBdryCurves) { for(ii=0;ii<4;ii++)
                                 { if(cpOptBdryCurves->GetAt(ii)->GetOwner())
                                     { pEdge = (SmEdge *)(cpOptBdryCurves->GetAt(ii)->GetEdge()) ;
                                       if(pEdge->GetBrep()) {pBrep = pEdge->GetBrep() ; }
                              }  }   }
          smgfx_Erase() ;
          if(cpOptBdryCurves) 
            { smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
              smgfx_SetLook(2,3, 1,0,1) ; if(pEdge) pEdge->Draw() ; sm_GraphicsLoop() ;
              smgfx_SetLook(3,4, 0,0,1) ; if(cpOptBdryCurves->GetAt(0)) cpOptBdryCurves->GetAt(0)->Draw() ; sm_GraphicsLoop() ;
              smgfx_SetLook(3,4, 0,1,1) ; if(cpOptBdryCurves->GetAt(1)) cpOptBdryCurves->GetAt(1)->Draw() ; sm_GraphicsLoop() ;
              smgfx_SetLook(3,4, 0,1,0) ; if(cpOptBdryCurves->GetAt(2)) cpOptBdryCurves->GetAt(2)->Draw() ; sm_GraphicsLoop() ;
              smgfx_SetLook(3,4, 1,0,0) ; if(cpOptBdryCurves->GetAt(3)) cpOptBdryCurves->GetAt(3)->Draw() ; sm_GraphicsLoop() ;
            }
          smgfx_SetLook(2,3,.5,.8,1); for(ii=0;ii<crPoints.GetSize();ii++) { crPoints[ii].Draw() ; } sm_GraphicsLoop() ;
          smgfx_SetLook(5,6,1,.6,.5); for(ii=0;ii<=(ULONG)nt;ii++) { ((SmPoint3d&)(T[ii])).Draw() ; } sm_GraphicsLoop() ;
          sm_GraphicsLoop() ;
        }
#endif // SM_DEBUG_CODE

      // Re-arrange the curve array in the following order:
      // i=0=BOTTOM, i=1=RIGHT, i=2=TOP, i=3=LEFT  from  { Bot Top Lft Rgt }
      NL_CURVE *tmp = curs[3];
      curs[3] = curs[2];
      curs[2] = curs[1];
      curs[1] = tmp;
      bdys = curs;
    } // end optional boundary curves given branch
  else // optional boundary curves not given branch
    {
      // get uv Points for each P point by projecting to a plane.
      //   this works ok when the sample points come from a pretty planar surface.
      u = N_AllocReal1dArray(np,&nlStacks);
      v = N_AllocReal1dArray(np,&nlStacks);
      SmPoint3d sPnt(0.0,0.0,0.0);
      NL_POINT    Og;
      NL_VECTOR  X, Y, Z;
      COPY_XYZ(sPnt,Og);
      COPY_XYZ(sPnt,X);
      COPY_XYZ(sPnt,Y);
      COPY_XYZ(sPnt,Z);
      NL_PARAMETER  us = 0.0, ue = 1.0; // u-bounds
      NL_PARAMETER  vs = 0.0, ve = 1.0; // v-bounds

      // get uv points for each P point by projecting to a plane fitted to the sample points
      NL_SER(N_FitSrfCalcParams(P,np,             // Sample Points, sized:[np+1]
                               NL_NO,             // NL_NO = compute projection plane
                               Og,X,Y,Z,          // not used when computing projection plane
                               us,ue,vs,ve,       // output parameter range
                               u,v));             // output uv points for each P point
      T = P;
      nt = np;

    } // end map P to uv points by projecting to a plane branch

  // output surface U and V dir control point counts must be as large as U and V dir boundary control point counts
  // gwc:Bug151 - modified n,m computation to equal the max boundary curve control point count
  if(cpOptBdryCurves)
    {
      ULONG lBdryCPtCnt0 = (cpOptBdryCurves->GetAt(0) != NULL) ? cpOptBdryCurves->GetAt(0)->GetNumberControlPoints() : lMaxUCtrlPnts ;
      ULONG lBdryCPtCnt1 = (cpOptBdryCurves->GetAt(1) != NULL) ? cpOptBdryCurves->GetAt(1)->GetNumberControlPoints() : lMaxUCtrlPnts ;
      ULONG lBdryCPtCnt2 = (cpOptBdryCurves->GetAt(2) != NULL) ? cpOptBdryCurves->GetAt(2)->GetNumberControlPoints() : lMaxVCtrlPnts ;
      ULONG lBdryCPtCnt3 = (cpOptBdryCurves->GetAt(3) != NULL) ? cpOptBdryCurves->GetAt(3)->GetNumberControlPoints() : lMaxVCtrlPnts ;
      lMaxUCtrlPnts = smos_3Max(lMaxUCtrlPnts, lBdryCPtCnt0, lBdryCPtCnt1) ;
      lMaxVCtrlPnts = smos_3Max(lMaxVCtrlPnts, lBdryCPtCnt2, lBdryCPtCnt3) ;
    }

  // locals 
  NL_INDEX  n = lMaxUCtrlPnts-1;
  NL_INDEX  m = lMaxVCtrlPnts-1;
  NL_DEGREE p = (NL_DEGREE)lUDegree;
  NL_DEGREE q = (NL_DEGREE)lVDegree;
  NL_REAL  *wts = NULL;

  NL_FLAG uPeriodic = NL_TRUE;
  if(!bPeriodicU) uPeriodic = NL_FALSE;
  NL_FLAG vPeriodic = NL_TRUE;
  if(!bPeriodicV) vPeriodic = NL_FALSE;

  // Fit the sample points P, given a uv Point for each P point and optional boundary curves
  NL_SER(N_FitSrfLstSqDeformablePeriodic
  (T,                                  /* in : sample points, sized:[np+1] */
   wts,                                /* in : opt least squares weights, sized:[np+1], */
                                       /*      NULL to ignore, sized:[np+1] */
   u,                                  /* in : u param for every P point, sized:[np+1] */
   v,                                  /* in : v param for every P point, sized:[np+1] */
   nt,                                 /* in : size param, size of P, uu, and vv arrays */
   bdys,                               /* in : opt compatible bdry curves: 0=Bottom, 1=Right, 2=Top, 3=Left,  */
                                       /*      bdys[i] = NULL to ignore one or more curves, */
                                       /*      bdys = NULL to ignore all curves */
   n,                                  /* in : output sur highest ControlPoint index U */
   m,                                  /* in : output sur highest ControlPoint index V */
   p,                                  /* in : output sur degree U */
   q,                                  /* in : output sur degree V */
   NULL,                               /* in : opt sur KnotVector U, NULL=function computes a likely knot vector */
   NULL,                               /* in : opt sur KnotVector V, NULL=function computes a likely knot vector */
   
   dAlpha,                             /* in : Resistance to stretch weight term */
   dBeta,                              /* in : Resistance to bending weight term */          
   dStiffness,                         /* in : Scales stiffness matrix. Higher value lessens impact of alpha and beta */       
   
   uPeriodic,                          /* in : u periodic flag: NL_YES = periodic in u, NL_NO = not    */                       
   vPeriodic,                          /* in : v periodic flag: NL_YES = periodic in v, NL_NO = not    */   
   NL_FULL,                            /* in : memory flag: NL_FULL   = more memory, faster, */
                                       /*                   NL_SPARSE = less memory, slower  */
   NL_LUPIV,                           /* in : solver flag: NL_SVD=SingleValueDecomposition, (requires full or over samplind sampling) */
                                       /*                   NL_LUPIV=LU Decomposition with partial pivoting, allows sparse sampling */
   NL_YES,                             /* in : solver cntrl: NL_YES=use the aflg internal solver,        */
                                       /*                    NL_NO=use the user supplied callback solver */
   NULL,                               /*          | in : X MatrixPtr of A X = B eqn | X MatrixPtr of A X = B eqn                */
   &nlSurf,                            /* out: the approximated surface */
   &nlStacks));                        /* in : sur's memory stack */

  rpNewSurf = new (crContext) SmBSplineSurface((gw_SURFACE *)&nlSurf) ;  // gwc: was TRUE = run AssertValidAndHeal on New BSplineSurface
#ifdef SM_DEBUG_CODE
  if (bDebugMe) 
    {
      // dump and draw newSurf with input args
      rpNewSurf->Dump();
      ULONG ii ;
      SmEdge *pEdge = NULL ;
      SmBrep *pBrep = NULL ;
      if(cpOptBdryCurves) { for(ii=0;ii<4;ii++)
                             { if(cpOptBdryCurves->GetAt(ii)->GetEdge())
                                 { pEdge = (SmEdge *)(cpOptBdryCurves->GetAt(ii)->GetEdge()) ;
                                   if(pEdge->GetBrep()) {pBrep = pEdge->GetBrep() ; }
                          }  }   }

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(2,3, 1,0,1) ; if(pEdge) pEdge->Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,0) ; if(rpNewSurf) rpNewSurf->DrawUV(); sm_GraphicsLoop();

      if(cpOptBdryCurves) 
        { smgfx_SetLook(3,4, 0,0,1) ; if(cpOptBdryCurves->GetAt(0)) cpOptBdryCurves->GetAt(0)->Draw() ; sm_GraphicsLoop() ;
          smgfx_SetLook(3,4, 0,1,1) ; if(cpOptBdryCurves->GetAt(1)) cpOptBdryCurves->GetAt(1)->Draw() ; sm_GraphicsLoop() ;
          smgfx_SetLook(3,4, 0,1,0) ; if(cpOptBdryCurves->GetAt(2)) cpOptBdryCurves->GetAt(2)->Draw() ; sm_GraphicsLoop() ;
          smgfx_SetLook(3,4, 1,0,0) ; if(cpOptBdryCurves->GetAt(3)) cpOptBdryCurves->GetAt(3)->Draw() ; sm_GraphicsLoop() ;
        }
      smgfx_SetLook(3,4,.5,.8,1); for(ii=0;ii<crPoints.GetSize();ii++) { crPoints[ii].Draw() ; } sm_GraphicsLoop() ;
      smgfx_SetLook(6,7,1,.6,.5); for(ii=0;ii<=(ULONG)nt;ii++) { ((SmPoint3d&)(T[ii])).Draw() ; } sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // all done
  return SM_SUCCESS;

} // end SmBSplineSurface::ApproximateRandomPointsDeformable

/*******************************************************************//**
PURPOSE: This routine computes a b-spline surface approximation to 
            a random (non-NxM) set of points 

NOTES:
    Optionally, users may input a (non-rational) boundary curve.
    The points and/or the curve must be supplied. If only the points
    are input, then the curve is computed and returned. If only the
    curve is input, the points are sampled off the curve and used.
    
Example:
    The specified lMaxUCtrlPnts & lMaxVCtrlPnts can make great difference
    in the results. Users are suggested to choose numbers that may best 
    'describe' the size & shape of the data points and follow the criteria:

    (lMaxUCtrlPnts+1) * (lMaxVCtrlPnts+1) <= Total # of Data Points.

    For example, suppose we have 100 sample points, then choose 8x8 or 7x7
    if a 'square-like' shape of surface is approximated. Other choices such as:
    9x7, 10x5,...etc. can be used to describe a 'rectangle-like' surfaces.

***********************************************************************/
SmStatus SmBSplineSurface::ApproximateRandomPoints
  (const SmContext              & crContext,       // in : New Object Context
   SmTArray<SmPoint3d>          * cpOpt3DPoints,   // in : Optional target Points
   SmBSplineCurve              *& cpOpt3DCurve,    // in : Optional boundary curve through crPoints
   ULONG                          lMaxUCtrlPnts,   // in : Output surface controlPoints U maximum count 
   ULONG                          lMaxVCtrlPnts,   // in : Output surface controlPoints V maximum count 
   ULONG                          lUDegree,        // in : Output surface degree U
   ULONG                          lVDegree,        // in : Output surface degree V
   SmBSplineSurface            *& rpNewSurf)       // out: approximated surface
{
  // check input
  SM_ASSERT( cpOpt3DPoints != NULL || cpOpt3DCurve != NULL ) ;

  // if points do not exist, then sample boundary curve to get points
  SmTArray<SmPoint3d> sBoundaryPoints;
  SmExtent1d sRange;

  ULONG ii = 0;
  ULONG nPoints = 0;

  // when given Opt3DPoints - load the Opt3DPoints into the sample array
  if( cpOpt3DPoints ) 
    {
      nPoints = cpOpt3DPoints->GetSize(); 
      for( ii = 0; ii < nPoints; ii++ ) 
        {
          sBoundaryPoints.Add( (*cpOpt3DPoints)[ii] );
        }
    }
  else // without Opt3DPoints - sample the Opt3dCurve 
    {
      sRange = cpOpt3DCurve->GetNaturalInterval();
      
      nPoints = 20;   // arbitrary
      double param = sRange.GetMin();
      double paramIncr = ( sRange.GetMax() - sRange.GetMin() ) / nPoints;

      // load cpOpt3DCurve sample points into BoundaryPoints array
      for( ii = 0; ii < nPoints; ii++ ) 
        {
          SmPoint3d point;
          cpOpt3DCurve->EvaluatePoint( param, point );
          sBoundaryPoints.Add( point );
          param += paramIncr;
        }
    }

  // if curve does not exist, then create curve from points
  if( cpOpt3DCurve == NULL ) 
    {
      SmBSplineCurve::ApproximatePoints( crContext, *cpOpt3DPoints, 3, NULL, NULL, TRUE, NULL, cpOpt3DCurve );
    }

  // Make sure the curve was created before moving on
  if( cpOpt3DCurve == NULL ) 
    {
      return( SM_ERR );
    }

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  if (bDebugMe) 
    {
      smgfx_Erase();
      smgfx_SetColor(0,0,1);
      cpOpt3DCurve->Draw();
      for( ii = 0; ii < nPoints; ii++ ) 
        {
          sBoundaryPoints[ii].Draw();
        }
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // memory stacks for Nlib routines 
  NL_STACKS  S;  
  N_InitNurbs(&S);

  // Copy points into NLib array
  NL_POINT* NPoints = N_AllocPt1dArray( nPoints, &S );
  for( ii = 0; ii < nPoints; ii++ ) 
    {
      N_PtFromXYZ( sBoundaryPoints[ii].x, sBoundaryPoints[ii].y, sBoundaryPoints[ii].z, &NPoints[ii] );
    }

  // Establish normals along curve
  NL_POINT* Normals = N_AllocPt1dArray( nPoints, &S );

  SmPoint3d sPt1(0,0,0), sPt2(0,0,0), sPt3(0,0,0);
  SmVector3d sVec1(0,0,0), sVec2(0,0,0), tmpNormal(0,0,0), sCrvNormal(0,0,0);
  double dT = 0.0, dTIncr;

  sRange = cpOpt3DCurve->GetNaturalInterval();
  cpOpt3DCurve->EvaluatePoint( sRange.Evaluate(dT), sPt3 );

  dTIncr = 1.0/nPoints;

  // for every BoundaryPoint - set normal= crossproduct of the vectors between this, the last, and the next points.
  for( ii = 0; ii < nPoints; ii++ ) 
    {
      sPt1  = sPt2;
      sPt2  = sPt3;
      sVec1 = sVec2;
      cpOpt3DCurve->EvaluatePoint( sRange.Evaluate( dT + dTIncr ), sPt3 );
      sVec2 = sPt3 - sPt2;
      dT += dTIncr;
      if( dT > 1.0 ) { dT = 1.0; }

      // skip first iteration - sVec1 is not yet set and assume this is a closed curve so last and first points are the same
      if( ii == 0 ) 
        { continue ; }
      
      // normal = crossproduct of the vectors between this, the last, and the next points.
      tmpNormal = sVec1 * sVec2;
      tmpNormal.Unitize();

      Normals[ii].x = tmpNormal.x;
      Normals[ii].y = tmpNormal.y;
      Normals[ii].z = tmpNormal.z;
    }

  // Create surface larger than boundary given points
  NL_SURFACE srf;
  N_SrfInitArrays(&srf);

  NL_DEGREE uDegree = (NL_DEGREE)lUDegree;
  NL_DEGREE vDegree = (NL_DEGREE)lVDegree;
  NL_FLAG nl_flag = N_FitRandomPN( NPoints, Normals, nPoints-1, lMaxUCtrlPnts, lMaxVCtrlPnts, uDegree, vDegree, &srf, &S );
  if( nl_flag == NL_YES ) 
    {
      return( SM_ERR );
    }

  rpNewSurf = new (crContext) SmBSplineSurface((gw_SURFACE*)&srf) ;  // gwc: was TRUE = run AssertValidAndHeal on New BSplineSurface

#ifdef SM_DEBUG_CODE
  if (bDebugMe) 
    {
      smgfx_Erase();
      smgfx_SetColor(1,0,0);
      rpNewSurf->Draw();
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // free up NLib data stacks
  N_EndNurbs(&S);

  return( SM_SUCCESS );

} // end SmBSplineSurface::ApproximateRandomPoints

/*******************************************************************//**
PURPOSE: This routine computes a least squares b-spline surface 
    approximation to a random (non-NxM) set of points. i.e. point cloud

NOTES:
    
Example:
    The specified lMaxUCtrlPnts & lMaxVCtrlPnts can make great difference
        in the results. Users are suggested to choose numbers that may best 
        'describe' the size & shape of the data points and follow the criteria:

            (lMaxUCtrlPnts+1) * (lMaxVCtrlPnts+1) <= Total # of Data Points.

        For example, suppose we have 100 sample points, then choose 8x8 or 7x7
        if a 'square-like' shape of surface is approximated. Other choices such as:
        9x7, 10x5,...etc. can be used to describe a 'rectangle-like' surfaces.
    
***********************************************************************/
SmStatus SmBSplineSurface::ApproximateRandomPointsNormals
  (const SmContext            & crContext,           // in : new object context
   const SmTArray<SmPoint3d>  & crPoints,            // in : sample points      
   const SmTArray<SmVector3d> & crNormals,           // in : associated surface normals
   ULONG                        lMaxUCtrlPnts,       // in : output control point count U 
   ULONG                        lMaxVCtrlPnts,       // in : output control point count V 
   ULONG                        lUDegree,            // in : output degree U
   ULONG                        lVDegree,            // in : output degree V
   SmBSplineSurface          *& rpNewSurf,           // out: approximating surface
   SmAxis2Placement           * pOptProjectionPlane, // in : Opt projection plane, NULL to ignore
                                                     //      default:[NULL]
   double                       dOptTolerance)       // in : Max distance allowed between approx and points, 
                                                     //      0.0 to ignore, default:[0.0]
{
  // init output
  SM_ASSERT(rpNewSurf == NULL) ;
  rpNewSurf = NULL ;

  // locals
  ULONG i; 
  NL_STACKS nlStacks;
  SmNLibStackHandler sSH(&nlStacks);
  NL_SURFACE sur;
  N_SrfInitArrays(&sur);

  // check input
  SM_ASSERT(crPoints.GetSize() > 0);
  SM_ASSERT(crNormals.GetSize() > 0);
  SM_ASSERT(crPoints.GetSize() == crNormals.GetSize()) ;

  // memory for Point and Normal samples
  NL_INDEX     np = crPoints.GetSize()-1;
  NL_POINT * P = N_AllocPt1dArray(np,&nlStacks);
  NL_POINT * N = N_AllocPt1dArray(np,&nlStacks);

  // for every sample point - load the P array
  for (i=0; i<crPoints.GetSize(); i++) 
    {
      SmPoint3d sPnt  = crPoints[i];
      SmPoint3d sNorm = crNormals[i];
      COPY_XYZ(sPnt,  P[i]);
      COPY_XYZ(sNorm, N[i]);

    } // end iter every input point

  // Set NLIb call parameters
  NL_INDEX  n = lMaxUCtrlPnts-1;
  NL_INDEX  m = lMaxVCtrlPnts-1;
  NL_DEGREE p = (NL_DEGREE)lUDegree;
  NL_DEGREE q = (NL_DEGREE)lVDegree;

  // optional projection plane - NULL to ignore
  NL_POINT  sOrigin, * pOrigin = NULL ; 
  NL_VECTOR sX,      * pX      = NULL ; 
  NL_VECTOR sY,      * pY      = NULL ; 

  // when given OptProjPlane - copy it into NLib parameters
  if(pOptProjectionPlane != NULL)
    {
      COPY_XYZ(pOptProjectionPlane->GetOriginRef(), sOrigin) ;
      COPY_XYZ(pOptProjectionPlane->GetXAxisRef(),  sX     ) ;
      COPY_XYZ(pOptProjectionPlane->GetYAxisRef(),  sY     ) ;

      pOrigin = &sOrigin ;
      pX      = &sX ;
      pY      = &sY ;

    } // end pOptProjectionPlane existence check 

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  if (bDebugMe) // Draw input
    {
      if(rpNewSurf) rpNewSurf->Dump();
      crPoints.Dump() ;
      crNormals.Dump() ;

      smgfx_Erase() ;
      ULONG ii, lPause = 30, lCnt = crPoints.GetSize() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pOptProjectionPlane) pOptProjectionPlane->Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 0,1,0) ; for(ii=0;ii<lCnt;ii++) { if(0==(ii%lPause) && ii != 0) { sm_GraphicsLoop(); 
                                                                                           continue ; 
                                                                                         } 
                                                           crPoints[ii].Draw() ; 
                                                         } sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 1,0,0) ; for(ii=0;ii<lCnt;ii++) { if(0==(ii%lPause) && ii != 0) { sm_GraphicsLoop(); 
                                                                                           continue ; 
                                                                                         } 
                                                           crNormals[ii].Draw(&crPoints[ii]) ; 
                                                         } sm_GraphicsLoop() ;
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // pass the call along
  NL_SER(N_FitRandomPNWithPlaneToTol( P,             /* Sample Points to fit, sized:[np+1] */
                                      N,             /* Sample Point Normals, sized:[np+1] */
                                      np,            /* highest index of arrays P and N    */
                                      dOptTolerance, /* max dist between points P and approx srf, 0.0 = ignore */
                                      pOrigin,       /* optional origin of projection plane, NULL=compute plane, NULL to ignore. */
                                      pX,            /* optional X axis of projection plane, NULL to ignore. */
                                      pY,            /* optional Y axis of projection plane, NULL to ignore. */
                                      n,             /* Output Surface U dir control point count */
                                      m,             /* Output Surface V dir control point count */
                                      p,             /* Output Surface U dir degree */
                                      q,             /* Output Surface V dir degree */
                                      &sur,          /* Output Surface */
                                      &nlStacks ));  /* sur memory stack */

  rpNewSurf = new (crContext) SmBSplineSurface((gw_SURFACE *)&sur) ;  // gwc: was TRUE = run AssertValidAndHeal on New BSplineSurface

#ifdef SM_DEBUG_CODE
  if (bDebugMe) 
    {
      ULONG ii, lCnt = crPoints.GetSize() ;
      if(rpNewSurf) rpNewSurf->Dump();

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pOptProjectionPlane) pOptProjectionPlane->Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 0,1,0) ; for(ii=0;ii<lCnt;ii++) { crPoints[ii].Draw() ; } sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 1,0,0) ; for(ii=0;ii<lCnt;ii++) { SmVector3d pt = crPoints[ii]; crNormals[ii].Draw(&pt) ; } sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,1) ; rpNewSurf->DrawUV() ; sm_GraphicsLoop() ;
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  return SM_SUCCESS;

} // end SmBSplineSurface::ApproximateRandomPointsNormals 

/*******************************************************************//**
PURPOSE: This approximation routine approximates the given NURBS
    surface with a non-rational surface of selected degrees.

NOTES: This method is only available for users with NLib
    
***********************************************************************/
SmStatus SmBSplineSurface::ApproximateRationalSurface
  (const SmContext   & crContext,           // in : 
   double              dThisApproxTol3d,    // in : 
   SmBSplineSurface *& rpNewSurf)           // out: 
{
    NL_STACKS nlStacks;
    SmNLibStackHandler sSH(&nlStacks);
    NL_SURFACE sur;
    N_SrfInitArrays(&sur);

    NL_SURFACE * surP = GetOrCreateGwNurbPointer() ;
    NER(surP);

    if (!IsRational()) {
        SER(SM_ERR);
    }

    NL_DEGREE  deg = 3;
    NL_SER(N_ApproxNurbsWithNonRatSrf(surP,dThisApproxTol3d,deg,deg, NL_INHERITED,&sur,&nlStacks));

    rpNewSurf = new (crContext) SmBSplineSurface((gw_SURFACE *)&sur) ; // gwc: was TRUE = run AssertValidAndHeal on New BSplineSurface
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
    if (bDebugMe) {
        sm_GraphicsLoop();
        smgfx_SetColor(0,0,1);
        rpNewSurf->DrawUV(1,1);
        rpNewSurf->Dump();
    }
#endif

    return SM_SUCCESS;

} // end SmBSplineSurface::ApproximateRationalSurface

/*******************************************************************//**
PURPOSE: Approximate a skinned surface from cross section curves
    with optional periodicity

NOTES: This method is only available for users with NLib

  lDegree can not be greater than number of cross-section curves.

  Cross sections curves must all go in the same direction.

METHOD --- converts input data forms and passes call along to Nlib
***********************************************************************/
SmStatus SmBSplineSurface::ApproximateSkinnedSurface
  (const SmContext                  & crContext,             // in : context for new geometry construction
   const SmTArray<SmBSplineCurve *> &crCrossSectionCurves,   // in : CrossSectionCurves in order of occurrence on surface
   SmBoolean                         bSyncronized,           // in : TRUE  = CrossSectionCurves have same degree and are synchronized
   SmBoolean                         bPeriodic,              // in : TRUE  = create NL_C1 smoothly closed surface if crCrossSectionCurves[0]=crCrossSectionCurves[k]
                                                             //      FALSE = do not force NL_C1 closure if crCrossSectionCurves[0]=crCrossSectionCurves[k]
   SmSurfParamType                   eCrossSectionSurfParam, // in : SM_SP_U/SM_SP_V = CrossSectionCurves become surface U/V isoparameter curves.
   double                            dThisApproxTol3d,       // in : Approximation tolerance for surface
   const SmTArray<double>          * pCrossSectionParams,    // in : Optional parameter value for each CrossSection Curve   
   SmBSplineSurface               *& rpNewBSplineSurface,    // out: The skinned surface
   ULONG                             lDegree)                // in : sweep direction degree, default:[3]
{
  // locals
  ULONG lNumSections = crCrossSectionCurves.GetSize();
  SmTArray<void*> sCrossSections( lNumSections );
  NL_STACKS     nlStacks;
  SmNLibStackHandler sSH(&nlStacks);

  // We'll need a local non-const copy of this:
  SmTArray< double > sIntParams;
  SmTArray< double > *pIntParamsPtr = NULL;
  if ( pCrossSectionParams != NULL )
    {
      sIntParams    = *pCrossSectionParams;
      pIntParamsPtr = &sIntParams;
    }

  // init SURFACE structure
  NL_SURFACE nlSurf;
  N_SrfInitArrays(&nlSurf);

#ifdef SM_DEBUG_CODE
  TCHAR sBuff1[SM_TBLOCK_SIZE];
  SmBoolean bDebugMe     = FALSE;
  SmBoolean bDebugOutput1 = FALSE;
  if (bDebugMe) 
    {
      // dump, write and draw inputs - rail curves, crossSections and derivSrfs
      if(pCrossSectionParams) { SM_DUMP_TARRAY( (*pCrossSectionParams ) ); }
      if(pIntParamsPtr          ) { SM_DUMP_TARRAY( (*pIntParamsPtr           ) ); }
      for(ULONG di=0;di<crCrossSectionCurves.GetSize();di++)
        { crCrossSectionCurves[di]->Dump(); 
          if (bDebugOutput1) { smos_sprintf(sBuff1, _T("cross%d.txt"), (int)di);
                              crCrossSectionCurves[di]->WriteToFile(sBuff1);
                            }
        }

      smgfx_Erase() ;
      smgfx_SetLook(3,4, 1,0,0); for(ULONG dj=0;dj<crCrossSectionCurves.GetSize();dj++)
                                   { crCrossSectionCurves[dj]->Draw(); sm_GraphicsLoop() ; }
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // iter every crossSection curve building the sCrossSections array
  for( ULONG ii=0; ii<lNumSections; ii++) 
    {
      NL_CURVE *pCur = sm_CreateNlibCurve(crCrossSectionCurves[ii],nlStacks);
      sCrossSections.Add(pCur);

    } // end iter every crossSection curve building the sCrossSections Array


  // NLib locals
  NL_CURVE     ** nlCurv             = (NL_CURVE **)sCrossSections.GetDataArray();
  NL_PARAMETER  * nlIntParams        = pIntParamsPtr ? (NL_PARAMETER*)pIntParamsPtr->GetDataArray() : NULL ;
  NL_INDEX        nlNumCrossCurves   = lNumSections;
  NL_FLAG      nlDir              = (eCrossSectionSurfParam == SM_SP_U) ? NL_UDIR : NL_VDIR ;
  NL_FLAG      nlSync             = (bSyncronized) ? NL_NO : NL_YES ; // nlSync == NL_YES : make curves compatible
  NL_FLAG      nlPeriodic         = ( bPeriodic ) ? NL_YES : NL_NO;
  NL_DEGREE       nlDeg              = (NL_DEGREE)lDegree;
  NL_KNOTVECTOR * nlKnots            = NULL;

  // If there are no additional constraints, then use the lowest degree
  // that will interpolate the given number of cross curves in a single span.
  // [5/28/10]
  if ( lDegree >= (ULONG) nlNumCrossCurves )
  { nlDeg = (NL_DEGREE) ( nlNumCrossCurves - 1 ); }

  // when pIntersectionsWithRails is given
  if (nlIntParams != NULL) 
  {
      nlKnots = N_AllocKnotVectorAndArray( nlNumCrossCurves + nlDeg, &nlStacks );
      N_FitCalcKnotVectorCrvApprox( nlIntParams, nlNumCrossCurves - 1, nlNumCrossCurves - 2 * nlDeg, nlDeg, nlKnots );
  }
  
  //  skin crossSections with ...
  if ( nlIntParams == NULL
     || NL_YES == N_CreateSkinSrfApproxParams( nlCurv,
                                               nlNumCrossCurves - 1,
                                               nlSync,
                                               nlPeriodic,
                                               nlNumCrossCurves - 2 * nlDeg, // hic
                                               nlDeg,
                                               nlDir,
                                               nlIntParams,
                                               nlKnots,
                                               &nlSurf, &nlStacks, &nlStacks ) )
  {
      if ( NL_YES == N_CreateSkinSrfApproxTol( nlCurv,
                                               nlNumCrossCurves - 1,
                                               nlSync,
                                               nlPeriodic,
                                               nlDeg - 1,
                                               nlDeg,
                                               SM_EFF_ZERO_PARAM,
                                               dThisApproxTol3d,
                                               nlDir,
                                               &nlSurf, &nlStacks, &nlStacks ) )

      {
          if ( NL_YES == N_CreateSkinSrfApprox( nlCurv,
                                                nlNumCrossCurves - 1,
                                                nlSync,
                                                nlPeriodic,
                                                nlNumCrossCurves - 2 * nlDeg,
                                                nlDeg,
                                                nlDir,
                                                &nlSurf, &nlStacks, &nlStacks ) )
          {
              SER( SM_ERR );
          } // end CreateSkinSrfApprox with minimal arguments
      } // end CreateSkinSrfApproxTol  with tolerance defined
  } // end CreateSkinSrfApproxParams   with CrossSection Curve parameters
  // set output
  rpNewBSplineSurface = new (crContext) SmBSplineSurface((gw_SURFACE *)&nlSurf) ;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugCheckSurface = FALSE;
  if (bDebugCheckSurface)
    {
      // Check for knot validity of surface
      SmValidityCheckType eValidityCheckThatFailed;
      if ( ! rpNewBSplineSurface->PassesValidityCheck(SM_VC_ALL,eValidityCheckThatFailed)) 
        { SE(SM_ERR_INVALID_INPUT); }
    }
#endif // SM_DEBUG_CODE

#ifdef SM_DEBUG_CODE
  if (bDebugMe) 
    {
      SmBoolean bDebugOutput2 = FALSE;
      rpNewBSplineSurface->Dump();
      if (bDebugOutput2)
        { rpNewBSplineSurface->WriteToFile(_T("surf.txt")); }

      // draw output
      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,1,1); rpNewBSplineSurface->DrawUV(1,1); sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,1); rpNewBSplineSurface->DrawNet(); sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,1); rpNewBSplineSurface->DrawPolygon(); sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 1,0,0) ; rpNewBSplineSurface->DrawAlong(SM_SP_U, 0.0, 1) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 1,0,0) ; rpNewBSplineSurface->DrawAlong(SM_SP_U, 1.0, 1) ; sm_GraphicsLoop() ;
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // all done
  return SM_SUCCESS;

} // end SmBSplineSurface::CreateSkinnedSurface

/*******************************************************************//**
PURPOSE: This routine approximates a subsurface bounded by 4 curves.

NOTES: 
     That is, given a surface and 4 curves lying on the  surface and
     forming the 4 boundaries of a subsurface, this routine creates
     a surface which approximates the implied subsurface on its interior
     and has the 4 curves as its boundaries.  

     This method is only available for users with NLib
     
***********************************************************************/
SmStatus SmBSplineSurface::ApproximateSubSurface
  (const SmContext                 & crContext,         // in : 
   const SmTArray<SmBSplineCurve*> & cr3DCurves,        // in : 
   const SmTArray<SmBSplineCurve*> & crUVCurves,        // in :
   SmBSplineSurface               *& rpNewSurf)         // out: 
{
    NL_STACKS nlStacks;
    SmNLibStackHandler sSH(&nlStacks);
    NL_SURFACE subsur;
    N_SrfInitArrays(&subsur);

    NL_SURFACE * surP = GetOrCreateGwNurbPointer() ;
    NER(surP);

    NL_CURVE  *curS3d[2], *curT3d[2], *curS2d[2], *curT2d[2];
    NL_INDEX i;
    for (i=0; i<4; i++) {
        if (cr3DCurves[i]->IsRational() || crUVCurves[i]->IsRational()) {
            SER(SM_ERR);
        }
        NL_CURVE * cur3d = cr3DCurves[i]->GetOrCreateGwNurbPointer();
        NL_CURVE * cur2d = crUVCurves[i]->GetOrCreateGwNurbPointer();
        if (i < 2) {
            curS3d[i] = cur3d;
            curS2d[i] = cur2d;
        }
        else {
            curT3d[i-2] = cur3d;
            curT2d[i-2] = cur2d;
        }
    }

    NL_SER(N_ApproxSubSrfWithSrf(curS3d,curT3d,curS2d,curT2d,surP,&subsur,NULL,NULL,&nlStacks));

    rpNewSurf = new (crContext) SmBSplineSurface((gw_SURFACE *)&subsur) ;
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
    if (bDebugMe) {
        sm_GraphicsLoop();
        smgfx_SetColor(0,0,1);
        rpNewSurf->DrawUV(1,1);
        rpNewSurf->Dump();
    }
#endif

    return SM_SUCCESS;

} // end SmBSplineSurface::ApproximateSubSurface

/*******************************************************************//**
PURPOSE: This routine exposes N_FitSrfApproxRemoveKnots

NOTES: This method is only available for users with NLib
     
***********************************************************************/
SmStatus SmBSplineSurface::ApproximateSurfaceRemoveKnots
( const SmContext         & crContext,        // in : 
  const SmTArray<double > & crUParams,        // in : u param values at which error is checked
  const SmTArray<double>  & crVParams,        // in : v param values at which error is checked
  SmSurfParamType           eDir,             // in : target flag: SM_SP_U    = remove knots in u-direction   
                                              //                   SM_SP_V    = remove knots in v-direction   
                                              //                   SM_SP_BOTH = remove knots in both direction
  const SmApproxTol3d     & crApproxTol,      // in : error tolerance
  SmBSplineSurface       *& rpNewSurf )       // out: new surface


{
    // init return
    SmStatus eStat = SM_SUCCESS;

    // Copy this surface, since N_FitSrfApproxRemoveKnots edits in place
    SmSurface * pSurface;
    Copy( crContext, pSurface );
    rpNewSurf = SM_CAST_PTR( SmBSplineSurface, pSurface );

    // Init NLib vars
    NL_STACKS               nlStacks;
    SmNLibStackHandler   sSH( &nlStacks );
    NL_REAL            * uParams = crUParams.GetDataArray();
    NL_REAL            * vParams = crVParams.GetDataArray();
    NL_SURFACE         * sur = rpNewSurf->GetOrCreateGwNurbPointer();
    NL_INDEX             mu = crUParams.GetSize() - 1;
    NL_INDEX             mv = crVParams.GetSize() - 1;
    NL_REAL           ** errMat;
    NL_FLAG              nlDir = ( eDir == SM_SP_BOTH ) ? NL_UVDIR :
                                 ( eDir == SM_SP_U )    ? NL_UDIR  : NL_VDIR;

    errMat = N_AllocReal2dArray( mu + 1, mv + 1, &nlStacks );

    // Pass to NLib to do the work
    if ( NL_YES == N_FitSrfApproxRemoveKnots( sur,         //   i/o: target surface */  
                                              uParams,     //   in : u param values at which error is checked, sized:[mu+1]
                                              vParams,     //   in : v param values at which error is checker, sized:[mv+1]
                                              errMat,      //   out: error matrix for each check point, sized:[mu+1][mv+1] 
                                              mu,          //   in : highest index in uParams */
                                              mv,          //   in : highest index in vParams */
                                              crApproxTol, //   in : error tolerance */
                                              nlDir ) )    //   in : target flag: NL_UDIR  = remove knots in u-direction   
                                                           //                     NL_VDIR  = remove knots in v-direction   
                                                           //                     NL_UVDIR = remove knots in both direction
        eStat = SM_ERR;

    return eStat;

} // SmBSplineSurface::ApproximateSurfaceRemoveKnots

/*******************************************************************//**
PURPOSE: This approximation routine refits the given NURBS surface to the
         given points, with optional fixed edges.

NOTES: This method is only available for users with NLib.
       "This" surface is modified in place.

       Exposes N_FitSurfApproxPoints().

ARGUMENTS: All arguments except the first are optional.
   crApproxPts       in: points to approximate more closely
   bFixEdgesU        in: if TRUE, do not allow the u-edges to move.
                         The u-edges are where u varies, i.e., constant v.
   bFixEdgesV        in: same for v edges.
   dAlpha0, dAlpha1  in: start and end values of alpha regularization term; see notes below
                         Default: -1 for both: allow the algorithm to decide; see notes below
   pErrors           out: resulting fit errors at each point
                         Default: NULL

Notes: regularization terms
   Regularization helps to maintain a fair curve, and is used because
   in theory, the very "best" curve for the data, in terms of minimizing
   the distance to each data point, would be full of kinks and loops.
   Alpha tends to minimize the area of the resulting surface, which avoids loops.

   Alpha is passed in as a starting value and an ending value, because it
   works best when it decreases as the iteration proceeds.  If values are
   passed in negative, default values will be used.  Smaller values will
   result in a better fit, but the curve might develop kinks or loops.
   The values should not be large: the current defaults are
   alpha_0 = 0.005 and alpha_1 = 0.001.

   Unfortunately, the regularization is specific to bicubic surfaces.
   If the input surface is not bicubic, regularization will not be
   applied, and the resulting surface might not be "fair".

   More details can be found in the documentation of the NLib routine N_FitSurfApproxPoints().

***********************************************************************/
SmStatus SmBSplineSurface::ApproximateRefit( const SmTArray< SmPoint3d > & crApproxPts,
                                                   SmBoolean bFixEdgesU,
                                                   SmBoolean bFixEdgesV,
                                                   double dAlpha0,
                                                   double dAlpha1,
                                                   SmTArray< double > * pErrors )
{
  // locals
  NL_SURFACE * sur = GetOrCreateGwNurbPointer();  NER( sur );

  NL_POINT *xPts  = SM_REINTERPRET_CAST( NL_POINT*, crApproxPts.GetDataArray() );
  NL_INDEX xPtsCount = crApproxPts.GetSize() - 1;

  NL_FLAG fixEdgesU = ( bFixEdgesU == FALSE ) ? 0 : 1;
  NL_FLAG fixEdgesV = ( bFixEdgesV == FALSE ) ? 0 : 1;

  NL_REAL *errors = NULL;
  if ( pErrors != NULL )
  {
      if ( pErrors->GetSize() < crApproxPts.GetSize() )
      {
          pErrors->SetSize( crApproxPts.GetSize() );
      }
      errors = SM_REINTERPRET_CAST( NL_REAL*, pErrors->GetDataArray() );
  }

  NL_FLAG nlibStat = N_FitSurfApproxPoints(xPts, 
                                           xPtsCount,
                                           sur, 
                                           fixEdgesU, 
                                           fixEdgesV,
                                           dAlpha0, 
                                           dAlpha1, 
                                           errors) ;

  return ( nlibStat == NL_NO ) ? SM_SUCCESS : SM_ERR;

} // end ApproximateRefit

/*******************************************************************//**
PURPOSE: This approximation routine approximates the given NURBS
    surface by evaluating points and using those points to approximate a new surface.

NOTES: This method is only available for users with NLib
       "This" surface is left unaffected
***********************************************************************/
SmStatus SmBSplineSurface::RebuildSurface
 ( const SmContext   & crContext,  // in : new object context 
   double            * dOptTol,    // in : NULL = use N_FitSrfLstSqApprox() to approximate
                                   //      a (lNumRows)x(lNumCols) controlPoint surface to crPoints
                                   //      NotNULL = use N_FitSrfApproxTol() to approximate
                                   //      a possible smaller controlPoint count surfact to crPoints
                                   //      to within specified tolerance.  Larger tolerances yield
                                   //      smaller control point surfaces.
   double            * dOptMaxDev, // out: max deviation of result from input surface
   SmBSplineSurface *& rpNewSurf,  // out: new approx surface
   ULONG               lNumPtsU,   // in, opt: number of control points in the u direction
   ULONG               lNumPtsV)   // in, opt: number of control points in the v direction
                                   //     default: 21
{
  // init output
  SM_ASSERT(rpNewSurf == NULL) ;
  rpNewSurf = NULL ;

  // Sanity check: just give them a minimum for cubics.
  if ( lNumPtsU < 4 ) { lNumPtsU = 4; }
  if ( lNumPtsV < 4 ) { lNumPtsV = 4; }

  // copy the this surface
  SmSurface* pCopy = NULL;
  this->Copy( crContext, pCopy );
  SmBSplineSurface* pSrf = (SmBSplineSurface*)pCopy;

  // reparameterize the copied surface
  pSrf->ReparametrizeWithArcLength();

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  if (bDebugMe) 
    {
      smgfx_SetDrawingMode(SM_DM_CROSSHATCH); smgfx_SetLook( 1,1, 0,1,0 );
      smgfx_Erase(); pSrf->Draw( ); sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // locals to Evaluate points on surface
  SmTArray<SmPoint3d> sPtsToFit;
  SmPoint3d pt;
  ULONG ii, jj;
  double dNumRows = lNumPtsU-1;
  double dNumCols = lNumPtsV-1;
 
  SmExtent2d sDomain = pSrf->GetNaturalUVDomain();
  SmVector2d uv;
  SmExtent1d dRangeU( sDomain.GetUInterval() );
  SmExtent1d dRangeV( sDomain.GetVInterval() );

  // Evaluate points on surface
  for( ii = 0; ii < lNumPtsU; ii++ ) 
    {
      uv.x = dRangeU.Evaluate( ii / dNumRows );
      for( jj = 0; jj < lNumPtsV; jj++ ) 
        {
          uv.y = dRangeV.Evaluate( jj / dNumCols );
          pSrf->EvaluatePoint( uv, pt );
          sPtsToFit.Add( pt );
        }
    }

#ifdef SM_DEBUG_CODE
  if (bDebugMe) 
    {
      smgfx_SetLook( 2,2, 1,0,0 );
      for( ii = 0; ii < sPtsToFit.GetSize(); ii++ ) 
        {
          sPtsToFit[ii].Draw( ); sm_GraphicsLoop();
        }
    }
#endif // SM_DEBUG_CODE

  // approx fit new surface to set of points
  SmBSplineSurface* pNewSrf = NULL;
  SmStatus stat = SmBSplineSurface::ApproximatePoints( crContext, sPtsToFit, lNumPtsU, lNumPtsU, 3, 3, dOptTol, pNewSrf );
  if( stat != SM_SUCCESS ) 
    { 
      return( stat );
    }

  if( pNewSrf == NULL ) 
    {
      return( SM_ERR );
    }

  // return surface
  rpNewSurf = pNewSrf;

#ifdef SM_DEBUG_CODE
  if (bDebugMe) 
    {
      smgfx_SetLook( 3,3, 0,0,1 ); pNewSrf->Draw( ); sm_GraphicsLoop();
    }
#endif

  // Check the max diff, if asked.
  if( dOptMaxDev != NULL ) 
    {
      *dOptMaxDev = 0.0;

      SmBoolean bSuccess;
      SmBoolean bIsMulti;
      SmPoint2d resultUV;
      double diff;
 
      for( ii = 0; ii < sPtsToFit.GetSize(); ii++ ) 
        {
          pNewSrf->DropPoint( sPtsToFit[ii], pNewSrf->GetNaturalUVDomain(), NULL, bSuccess, resultUV, diff, bIsMulti ) ;
          if( bSuccess && diff > *dOptMaxDev ) 
            {
              *dOptMaxDev = diff;
            }
        }
    }

  return( SM_SUCCESS );

} // end SmBSplineSurface::RebuildSurface

/*******************************************************************//**
PURPOSE: This routine computes the cross-boundary derivative curve of
    the NURBS surface. 

NOTES: This method is only available for users with NLib
    
***********************************************************************/
SmStatus SmBSplineSurface::CalculateCrossBoundaryDerivative
  (const SmContext & crContext,    // in : context for new object construction
   SmSurfParamType   eSurfDir,     // in : SM_SP_U,  NL_LEFT  : derivative across u=umin
                                   //    :           NL_RIGHT : derivative across u=umax
                                   //    : SM_SP_V,  NL_BOTTOM: derivative across v=vmin
                                   //    :           NL_TOP   : derivative across v=vmax
   SmBoolean         bIsAtMin,     // in : TRUE  - SM_SP_U=>NL_LEFT  and SM_SP_V=>NL_BOTTOM
                                   //    : FALSE - SM_SP_U=>NL_RIGHT and SM_SP_V=>NL_TOP
   SmBSplineCurve *& rpDerivCurve) // out: CrossDeriv curve
{
  NL_CURVE   cur;
  NL_STACKS  SS;
  SmNLibStackHandler sSH1(&SS);
  N_CrvInitArrays(&cur);

  // local Nurb pointer
  NL_SURFACE * surP = GetOrCreateGwNurbPointer() ;
  NER(surP);

  // convert input into NLib side flag value
  NL_FLAG side =  (eSurfDir == SM_SP_U)
             ? (bIsAtMin ? NL_LEFT   : NL_RIGHT)   // SM_SP_U
             : (bIsAtMin ? NL_BOTTOM : NL_TOP) ;   // SM_SP_V

  // Build the NLIb CrossDerivative curve
  NL_SER(N_CrossBoundDerivCrvNurbsSrf(surP,side,&cur,&SS));

  // Build SMLib BSPlineCurve from NLib gw_CURVE
  rpDerivCurve = new (crContext) SmBSplineCurve(3,(gw_CURVE *)&cur);

  // all done
  return SM_SUCCESS;

} // end SmBSplineSurface::CalculateCrossBoundaryDerivative

/*******************************************************************//**
PURPOSE: Convert the given surface to a power basis form.  

NOTES: It assumes that the surface is already in bezier form.
***********************************************************************/
SmStatus SmBSplineSurface::ConvertToPowerBasis
  ()
{
    NL_STACKS     nlStacks;
    SmNLibStackHandler sSH(&nlStacks);

    const SmContext *pContext = GetContext();
    SmBSplineSurface *pTemplate = new(*pContext) SmBSplineSurface(*this);
    SmObjDelete sClean(pTemplate);

    NL_SURFACE *pFrom = pTemplate->GetOrCreateGwNurbPointer();
    NL_SURFACE *pTo   = GetOrCreateGwNurbPointer();

    NL_RMATRIX pum, pvm;
    N_InitRealMatrix(&pum);
    N_InitRealMatrix(&pvm);
    if (pFrom->knu->m != pFrom->p*2 + 1) SER(SM_ERR); // Not a bezier;
    if (pFrom->knv->m != pFrom->q*2 + 1) SER(SM_ERR); // Not a bezier;
    NL_SER(N_BezSrfToPower(pFrom->net->Pw,
                           pFrom->p,
                           pFrom->q,
                           &pum,&pvm,
                           pFrom->knu->U[0],
                           pFrom->knu->U[pFrom->knu->m],
                           pFrom->knv->U[0],
                           pFrom->knv->U[pFrom->knv->m],
        pTo->net->Pw));

    return SM_SUCCESS;

} // end SmBSplineSurface::ConvertToPowerBasis

/*******************************************************************//**
PURPOSE: Create a 3D iso-parametric deriv-curve of a SmBSplineSurface 
    whose values are the surface's derivatives given the 
    Nurb parameter direction (U or V), the constant parameter 
    in that direction and the desired derivative type.

NOTES ---
  1. Based on NLib's N_KDerivSrf() function.
  2. lDerCnt_U and lDerCnt_V must be less than surface degrees for Non-Rational and
          less than surface degrees-1 for Rational surfaces
  3. Commonly used to create cross derivative curve data as input
      to creating NSided patches

EXAMPLE --- CrossDerivative values on the surface's left-side.
 Calling CreateIsoParametricDerivCurve() with
  lDerCnt_U     = 1
  lDerCnt_V     = 0
  eSurfParam    = SM_SP_U
  dIsoParameter = pSurf->GetNaturalUVDomain()->GetUMin() 
 sets rpNewIsoDerivCurve = u=0 constant IsoParameterCurve whose values
                           are the CrossDerivative values along the surface's left-side boundary.
 Since that's a starting side, these cross-derivatives point into the surface.
 Negating the evaluations yields cross-derivative values pointing out of the surface.

***********************************************************************/
SmStatus SmBSplineSurface::CreateIsoParametricDerivCurve
  (const SmContext  & crContext,             // in : context for created objects
   ULONG              lDerCnt_U,             // in : desired derivative in U direction (0=none,1=1st,2=2nd,...).
                                             //       less than degree p for NUBs and less than p-1 for NURBs.
   ULONG              lDerCnt_V,             // in : desired derivative in V direction (0=none,1=1st,2=2nd,...). 
                                             //       less than degree q for NUBs and less than q-1 for NURBs.
   SmSurfParamType    eSurfParam,            // in : Defines which parameter direction on surface to extract curve from
                                             //      SM_SP_U = create constant u isoParameter curve
                                             //      SM_SP_V = create constant v isoParameter curve
   double             dIsoParameter,         // in : Defines parametric value at which to extract the curve.  
                                             //      If eSurfParam==SM_SP_U this is a U parameter, if eSurfParam==SM_SP_V
                                             //      then this is the V parameter
   double             d3DTolerance,          // in : d3DTolerance = Not used in this method
   SmBSplineCurve  *& rpNewIsoDerivCurve,    // out: 3d IsoParameterCurve
   const SmExtent2d * pOptDomain)            // in : optional trim bound for IsoParameterCurve
                                             //      NULL=use GetNaturalUVDomain(), default:[NULL] 
 const
{
  // check input - NULL output pointer reference
  if( rpNewIsoDerivCurve != NULL)
    {
      return SM_ERR_NON_NULL_OUTPUT_POINTER ;
    }

  // check input - valid DerCnt values
  if(   (lDerCnt_U >= (GetDegree(SM_SP_U) - (IsRational() ? 1 : 0)))
     || (lDerCnt_V >= (GetDegree(SM_SP_V) - (IsRational() ? 1 : 0))))
    {
      return SM_ERR_INVALID_INPUT ;
    }

  // stack for NLib call
  NL_STACKS nlStacks;
  SmNLibStackHandler sSH(&nlStacks);
  
  // locals
  NL_SURFACE  sDer;
  NL_SURFACE *pSur = ((SmBSplineSurface *)this)->GetOrCreateGwNurbPointer(); 
  NER(pSur);
  NL_CURVE    sCur;
  NL_INDEX k    = lDerCnt_U ;
  NL_INDEX l    = lDerCnt_V ;
  N_SrfInitArrays(&sDer) ;
  N_CrvInitArrays(&sCur) ;
  
  // Have NLib build the derivative surface
  N_KDerivSrf(pSur, k, l, &sDer, &nlStacks) ;

  // make a temporary SMLib Surface
  SmBSplineSurface *pDerSurf = new (crContext) SmBSplineSurface(&sDer) ; // gwc: was TRUE = run AssertValidAndHeal on New BSplineSurface
  SmObjDelete sClean(pDerSurf) ;

  // Get the isoCurve
  return(pDerSurf->CreateIsoParametricCurve(crContext, 
                                            eSurfParam, 
                                            dIsoParameter, 
                                            d3DTolerance, 
                                            rpNewIsoDerivCurve, 
                                            pOptDomain)) ;

} // end SmBSplineSurface::CreateIsoParametricDerivCurve


/*******************************************************************//**
PURPOSE: This routine prepares a curve network to be interpolated
   by a surface.

NOTES: 
  The curves must all intersect properly, within tolerance.

  Input is SMLib curves, output is NLib curves.

  Arguments:
  dTol : tolerance used for: ApproximateRationalCurve(); IsClosed();
    DropPoint() or GlobalCurveIntersect().
    If not specified, base it on curve lengths.
  bMakeNonRational: if True, approximate rational curves with non-rational
  lNumPtsBetweenU: minimum number of control points on each V curve
    between each pair of U curves; similar for lNumPtsBetweenV.
  bCommonParamRanges: All curves in each direction will be reparameterized 
    to the same domains.
  bCommonParameterization: All curves in each direction will have the same
    parameter values at each intersection with the other-direction curves.
  u_arg: if bCommonParameterization, 1-dimensional array of parameters
    of intersections of v-curves along u-curves; similar for v_arg.
  uu_arg: 2-dimensional array of parameters of intersections of v-curves
    along each u-curve; similar for vv_arg.
    These optional arrays are on the stack, and point directly there.
    As such, they will be deleted when the stack is released.  If you want
    to retain them after the stack is released, they must be copied.

METHOD ---
- Calc tolerance if not given.
- Create an NLib curve for each input curve.
- If specified, approximate rational curves with non-rational.
- If specified, reparamterize each curve to a common domain.
- Make curves compatible: same degree, knot vector, rationality
- Check whether the curves are closed.
- Compute all curve/curve intersections: double loop
- If specified, map the intersection parameters to a common domain.
- If specified, make sure there are enough control points between
  each other-direction curves.
- If specified, reparameterize to have the same knot values at
  the intersections.

***********************************************************************/

SmStatus SmBSplineSurface::PrepCurvesForSurfacing
(
  const SmContext & crContext,                  // in :
  const SmTArray<SmBSplineCurve*> & crUCurves,  // in :
  const SmTArray<SmBSplineCurve*> & crVCurves,  // in :
  NL_CURVE * nlCurU[],                             // out:
  NL_CURVE * nlCurV[],                             // out:
  NL_STACKS &nlStacks,                             // in :
  double dTol,                                  // in : default: -1 (not specified)
  SmBoolean bMakeNonRational,                   // in : default False
  ULONG lNumPtsBetweenU,                        // in : default 0
  ULONG lNumPtsBetweenV,                        // in : default 0
  SmBoolean bCommonParamRanges,                 // in : default True
  SmBoolean bCommonParameterization,            // in : default True
  double * u_arg[],                             // in : default NULL
  double * v_arg[],                             // in : default NULL
  double ** uu_arg[],                           // in : default NULL
  double ** vv_arg[]                            // in : default NULL
)
{
    ULONG lUCount = crUCurves.GetSize();
    ULONG lVCount = crVCurves.GetSize();
    NL_PARAMETER ** uu = N_AllocReal2dArray(lVCount-1,lUCount-1,&nlStacks);
    NL_PARAMETER ** vv = N_AllocReal2dArray(lVCount-1,lUCount-1,&nlStacks);
    ULONG ii, jj;

    // If not given, compute dTol from average of center curve in u and v.
    if ( dTol <= 0.0 )
    {
        SmBSplineCurve * pUCurve = crUCurves[(crUCurves.GetSize()/2)];
        SmBSplineCurve * pVCurve = crVCurves[(crVCurves.GetSize()/2)];
        dTol = SM_EFF_ZERO_SQRT * (pUCurve->ApproximateLength(pUCurve->GetNaturalInterval(),5) +
                                   pVCurve->ApproximateLength(pVCurve->GetNaturalInterval(),5))*0.5 ;
    }

    // Create an NLib curve for each input curve.
    // Optionally, if the input curve is rational, approximate it with non-rational.
    // Optionally, also, make all curves (in each direction) have the same parameterization.
    // Make them all unit.  [B82]
    gw_INTERVAL unitIvl;
    unitIvl.ul = 0.0;
    unitIvl.ur = 1.0;

    // First do u-curves.
    for (ii=0; ii<lUCount; ii++)
    {
        SmBSplineCurve * pCurve = crUCurves[ii];
        pCurve->GetOrCreateGwNurbPointer() ;
        SM_ASSERT( pCurve->GetGwNurbPointer() != NULL );

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
        if (bDebugMe) {
            if (ii==0) smgfx_Erase(0); // debugger crash if not 0 arg.
            pCurve->Draw();
            sm_GraphicsLoop();
        }
#endif

        SmObjDelete sClean;
        if ( bMakeNonRational && pCurve->IsRational() )
        {
            SmBSplineCurve *pNewCurve = NULL;
            SER( pCurve->ApproximateRationalCurve( crContext, dTol, pNewCurve) );
            sClean.SetObj( pNewCurve );
            pCurve = pNewCurve;
        }
        nlCurU[ii] = sm_CreateNlibCurve( pCurve, nlStacks );

        // Reparameterize to common interval.
        if ( bCommonParamRanges )
          { N_BasisReparam( nlCurU[ii]->knt, nlCurU[ii]->p, unitIvl ); }

    } // end for each u-curve

    // Then do v-curves.
    for (ii=0; ii<lVCount; ii++)
    {
        SmBSplineCurve * pCurve = crVCurves[ii];
        pCurve->GetOrCreateGwNurbPointer() ;
        SM_ASSERT( pCurve->GetGwNurbPointer() != NULL );

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
        if (bDebugMe) {
            pCurve->Draw();
            sm_GraphicsLoop();
        }
#endif
        SmObjDelete sClean;
        if ( bMakeNonRational && pCurve->IsRational() )
        {
            SmBSplineCurve *pNewCurve = NULL;
            SER( pCurve->ApproximateRationalCurve( crContext, dTol, pNewCurve ));
            sClean.SetObj( pNewCurve );
            pCurve = pNewCurve;
        }
        nlCurV[ii] = sm_CreateNlibCurve( pCurve, nlStacks );

        // Reparameterize to a common domain.
        if ( bCommonParamRanges )
          { N_BasisReparam( nlCurV[ii]->knt, nlCurV[ii]->p, unitIvl ); }

    } // end for each v-curve


    // Make curves compatible.
    NL_SER( N_CrvsMakeCompatible( nlCurU, lUCount-1, &nlStacks ));
    NL_SER( N_CrvsMakeCompatible( nlCurV, lVCount-1, &nlStacks ));


    // We need to check whether the curves are closed in either direction.
    // We'll just check the first in each direction: they should all be the same.
    SmSurfParamType eClosedDir = SM_SP_NEITHER;
    if ( crUCurves[0]->IsClosed( crUCurves[0]->GetNaturalInterval(), dTol ) )
      { eClosedDir = SM_SP_U; }
    if ( crVCurves[0]->IsClosed( crVCurves[0]->GetNaturalInterval(), dTol ) )
      { eClosedDir = ( eClosedDir == SM_SP_U ) ? SM_SP_BOTH : SM_SP_V; }
    if ( eClosedDir == SM_SP_BOTH )
      { SER_MSG( SM_ERR_INVALID_INPUT, _T("Error: Curve Mesh Surface: Curves closed in both directions")); }

    // Compute intersections between u-curves & v-curves.
    // Some locals.
    SmPoint3d sPnt;
    double dDist, dParam;
    SmBoolean bSuccess;

    for ( jj=0; jj<lUCount; jj++ )
    {
        SmBSplineCurve * pUCurve = crUCurves[jj];

        SmExtent1d sUDomain = pUCurve->GetNaturalInterval();
        // So we don't have to recalculate this for each v-curve:
        SmBoolean bUDegenerate = pUCurve->IsDegenerate();

        for ( ii=0; ii<lVCount; ii++ )
        {
            SmBSplineCurve * pVCurve = crVCurves[ii];

            SmExtent1d sVDomain = pVCurve->GetNaturalInterval();

            // If either curve is degenerate, drop its position to the other curve
            // to get the other curve's parameter.
            // For the degenerate curve, prorate the parameter value.
            if ( bUDegenerate )
            {
                SER( pUCurve->EvaluatePoint( sUDomain.GetMin(), sPnt ));
                SER( pVCurve->DropPoint(sVDomain,  // in : target curve allowed domain
                                        sPnt,      // in : Point to drop to curve
                                        NULL,      // in : For Snapping, NULL=NoSnap, Vec pointing to CrvInside from desired EndPt for snapping.
                                                   //      when DropPt is within dDistTol in cpVecToCrvInside dir of EndPt, Snap to Ivl EndPt.
                                                   //      Vec handles ambiguities and makes sure that curves are not just touching at the ends.
                                        dTol,      // in : Used for SnapDist, and SM_SO_INTERSECT sXSectTol3d distance.
                                                   //      sXSectTol3d = dDistTol ;      SM_SO_INTERSECT 
                                                   //      sXSectTol3d = SM_BIG_DOUBLE ; SM_SO_MINIMIZE, SM_SO_NORMALIZE
                                                   //      sSnapTol3d  = dDistTol ;      SM_SO_MINIMIZE, SM_SO_NORMALIZE, SM_SO_INTERSECT    
                                        NULL,      // in : If given, call cheap LocalPointSolve() else call expensive GlobalPointSolve()
                                        bSuccess,  // out: TRUE = found a drop point
                                        dParam,    // out: found drop curve param
                                        dDist)) ;  // out: found drop distance
                                                   // in : SM_SO_MINIMIZE = allow nonNormal drops near endPoints
                                                   //      SM_SO_INTERSECT= like Minimize but point must be within dDistTol
                                                   //      SM_SO_NORMALIZE= exclude nonNormal drops near endPoints
                                                   //      default:[SM_SO_MINIMIZE] to preserve original behavior
                uu[ii][jj] = sUDomain.Evaluate( double(ii) / double(lVCount-1) );
                vv[ii][jj] = dParam;
                continue;
            }
            else if (pVCurve->IsDegenerate() && ii>0)
            {
                SER( pVCurve->EvaluatePoint( sVDomain.GetMin(), sPnt ));
                SER( pUCurve->DropPoint(sUDomain,  // in : target curve allowed domain
                                        sPnt,      // in : Point to drop to curve
                                        NULL,      // in : For Snapping, NULL=NoSnap, Vec pointing to CrvInside from desired EndPt for snapping.
                                                   //      when DropPt is within dDistTol in cpVecToCrvInside dir of EndPt, Snap to Ivl EndPt.
                                                   //      Vec handles ambiguities and makes sure that curves are not just touching at the ends.
                                        dTol,      // in : Used for SnapDist, and SM_SO_INTERSECT sXSectTol3d distance.
                                                   //      sXSectTol3d = dDistTol ;      SM_SO_INTERSECT 
                                                   //      sXSectTol3d = SM_BIG_DOUBLE ; SM_SO_MINIMIZE, SM_SO_NORMALIZE
                                                   //      sSnapTol3d  = dDistTol ;      SM_SO_MINIMIZE, SM_SO_NORMALIZE, SM_SO_INTERSECT    
                                        NULL,      // in : If given, call cheap LocalPointSolve() else call expensive GlobalPointSolve()
                                        bSuccess,  // out: TRUE = found a drop point
                                        dParam,    // out: found drop curve param
                                        dDist)) ;  // out: found drop distance
                                                   // in : SM_SO_MINIMIZE = allow nonNormal drops near endPoints
                                                   //      SM_SO_INTERSECT= like Minimize but point must be within dDistTol
                                                   //      SM_SO_NORMALIZE= exclude nonNormal drops near endPoints
                                                   //      default:[SM_SO_MINIMIZE] to preserve original behavior
                uu[ii][jj] = dParam;
                vv[ii][jj] = sVDomain.Evaluate( double(jj) / double(lUCount-1) );
                continue;
            }

            // Both curve non-degenerate: intersect them.
            SmSolution aData[4];
            SmSolutionArray sSolutions(4,aData);
            SER( pUCurve->GlobalCurveIntersect( sUDomain,
                *pVCurve, sVDomain, dTol*100.0, sSolutions ));

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe3 = FALSE;
            if (bDebugMe3) {
                sSolutions.Dump();
                if (jj==0 && ii==0) smgfx_Erase();
                smgfx_SetColor(1,0,0);
                pUCurve->DrawAt(pUCurve->GetNaturalInterval().GetMin(),0);
                pUCurve->Draw();
                pUCurve->Dump();
                sm_GraphicsLoop();
                smgfx_SetColor(0,0,1);
                pVCurve->DrawAt(pVCurve->GetNaturalInterval().GetMin(),0);
                pVCurve->Draw();
                pVCurve->Dump();
                sm_GraphicsLoop();
            }
#endif

            // The curves must intersect, else the surface can't be created.
            if (sSolutions.GetSize() < 1)
            {
                SER(SM_ERR);
            }

            // Closed curve gets two solutions, start and end.
            if ( sSolutions.GetSize() == 2 )
            {
                if ( eClosedDir == SM_SP_U )
                {
                  // Note: don't know which order the solutions are in: check param values.
                  // We should be going in increasing direction on both curves,
                  // so use the smaller param first, then the larger. [B88]
                    if (   (ii == 0  &&  sSolutions[1].m_vStart[0] < sSolutions[0].m_vStart[0])
                        || (ii >  0  &&  sSolutions[1].m_vStart[0] > sSolutions[0].m_vStart[0])
                       )
                      { sSolutions.RemoveAt(0); }
                    else
                      { sSolutions.RemoveAt(1); }
                }
                else if ( eClosedDir == SM_SP_V )
                {
                    if (   (jj == 0  &&  sSolutions[1].m_vStart[1] < sSolutions[0].m_vStart[1])
                        || (jj >  0  &&  sSolutions[1].m_vStart[1] > sSolutions[0].m_vStart[1])
                       )
                      { sSolutions.RemoveAt(0); }
                    else
                      { sSolutions.RemoveAt(1); }
                }
            }

            SmSolution & rSol = sSolutions[0];

            // Sometimes the global solver can return too many solutions.
            // Take the best if so. [B245]
            if ( sSolutions.GetSize() > 1 )
              {
                ULONG kk, lNumSols = sSolutions.GetSize();
                for ( kk=1; kk<lNumSols; kk++ )
                  {
                    if ( sSolutions[kk].m_eSolutionType != SM_ST_SINGLE_VALUE )
                      { continue; }
                    if ( rSol.m_eSolutionType != SM_ST_SINGLE_VALUE
                      || sSolutions[kk].m_vStart.m_dSolutionValue < rSol.m_vStart.m_dSolutionValue )
                      {
                        rSol = sSolutions[kk];
                      }
                  }
              } // end if multiple solutions after checking closed curves.

            if (rSol.m_eSolutionType != SM_ST_SINGLE_VALUE)
            {
                SER(SM_ERR);
            }

            uu[ii][jj] = rSol.m_vStart[0];
            vv[ii][jj] = rSol.m_vStart[1];

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe4 = FALSE;
            if (bDebugMe4) {
                if (FALSE) {
                    smgfx_Erase();
                }
                smgfx_SetColor(1,0,0);
                pUCurve->DrawAt(rSol.m_vStart[0],0);
                pUCurve->Draw();
                sm_GraphicsLoop();
                smgfx_SetColor(0,0,1);
                pVCurve->DrawAt(rSol.m_vStart[1],0);
                pVCurve->Draw();
                sm_GraphicsLoop();
            }
#endif

        } // end for ii each v-curve
    } // end for jj each u-curve.  Done getting all the curve-curve intersections.

    // Map the intersection parameters to the unit interval. [B82]
    if ( bCommonParamRanges )
    {
        for ( jj=0; jj<lUCount; jj++ )
        {
            double dU0 = uu[0][jj];
            double dUDelta = uu[lVCount-1][jj] - dU0;

            for ( ii=0; ii<lVCount; ii++ )
            {
                double dV0 = vv[ii][0];
                double dVDelta = vv[ii][lUCount-1] - dV0;

                uu[ii][jj] = ( uu[ii][jj] - dU0 ) / dUDelta;
                vv[ii][jj] = ( vv[ii][jj] - dV0 ) / dVDelta;
            }
        }
    } // end mapping intersection parameters if asked.

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
    if (bDebugMe) {
        for (ULONG jjj=0; jjj<lUCount; jjj++) {
            smos_WriteBuffer(_T("\n"));
            ULONG kkk;
            for (kkk=0; kkk<lVCount; kkk++) {
                TCHAR sBuff[SM_TBLOCK_SIZE];
                smos_sprintf(sBuff,_T("[%ld,%ld] uu=%16.16lf, vv=%16.16lf\n"),jjj,kkk,uu[kkk][jjj],vv[kkk][jjj]);
                smos_WriteBuffer(sBuff);
            }
        }
    }
#endif

    // For the SweepSurface, we have to have a couple of sample profiles in between
    // each given profile, otherwise we can get a crease at the given profiles.
    // We get a sample profile at each control point on the rails between each profile,
    // so we insert some if not enough.
//cbi But we have to do it before N_ReparmCrvsIsectPt() ???

//cbi ... this will now be an input param ...

    // Make sure there will be enough profiles.
    // We will create one profile for each control point along the rail.
    // If insufficient, insert knots.

    ULONG kk;
    if ( ULONG( nlCurU[0]->pol->n ) < lVCount * lNumPtsBetweenU )
    {
        // For each rail curve:
        for ( ii = 0; ii < lUCount; ii++ )
        {
            // For each span between profiles:
            for ( jj = 0; jj+1 < lVCount; jj++ ) // note: can't say lVCount-1
            {
                double t0 = uu[jj][ii];
                double dDeltaT = ( uu[jj+1][ii] - uu[jj][ii] ) / ( lNumPtsBetweenU+1 );
                if ( dDeltaT > SM_EFF_ZERO_SQRT )
                {
                    // For each param to be inserted:
                    for ( kk = 1; kk <= lNumPtsBetweenU; kk++ )
                    {
                        double dT = t0 + kk * dDeltaT;

                        N_CrvInsertKnot( nlCurU[ii], dT, 1, nlCurU[ii], &nlStacks, &nlStacks );
                    }
                }
            }
        }
    }
//cbi: CHECK INDEXING (3-rail sweep project)
    // Repeat for v-curves.
    if ( ULONG( nlCurV[0]->pol->n ) < lUCount * lNumPtsBetweenV )
    {
        // For each profile curve:
        for ( ii = 0; ii < lVCount; ii++ )
        {
            // For each span between rails:
            for ( jj = 0; jj+1 < lUCount; jj++ ) // note: can't say lUCount-1
            {
                double t0 = vv[ii][jj];
                double dDeltaT = ( vv[ii][jj+1] - vv[ii][jj] ) / ( lNumPtsBetweenV+1 );
                if ( dDeltaT > SM_EFF_ZERO_SQRT )
                {
                    // For each param to be inserted:
                    for ( kk = 1; kk <= lNumPtsBetweenV; kk++ )
                    {
                        double dT = t0 + kk * dDeltaT;

                        N_CrvInsertKnot( nlCurV[ii], dT, 1, nlCurV[ii], &nlStacks, &nlStacks );
                    }
                }
            }
        }
    }

    // Now we have the grids of intersection parameters in uu and vv.
    // If asked, reparameterize the two sets of curves, nlCurU and nlCurV,
    // to force compatible parameter values at the intersection points.

    NL_PARAMETER *uParam = NULL, *vParam = NULL;
    if ( bCommonParameterization )
    {
        NL_SER( N_ReparmCrvsIsectPt( nlCurU, nlCurV, lUCount-1, lVCount-1,
            (NL_PARAMETER **)uu, (NL_PARAMETER **)vv, NL_LINEAR, &uParam, &vParam, &nlStacks, &nlStacks ));
    }

//cbi new:
// Can't do this: srf doesn't interpolate profile curves.
int cbi2 = 0;
if ( cbi2 == 1 ) {
    // The 'prepared' curves have full internal knot multiplicity
    // at their intersections.  (N_ReparmCrvsIsectPt() does that.)
    // This will be propagated to the resulting surface, which can produce
    // creases.  Try to remove them.
    // First remove as many as possible, then make sure they're still compatible.
//cbi just the rails?
    for ( ii = 0; ii < lUCount; ii++ )
      {
        // sm_CrvRemoveKnots( nlCurU[ii], dTol, nlCurU[ii], &nlStacks );
        N_CrvRemoveAllKnots(  nlCurU[ii], dTol, nlCurU[ii], &nlStacks );
        // N_CrvRemoveKnotsDerivConstraints( nlCurU[ii], dTol, constraints, der, nlCurU[ii], &nlStacks );
      }
    for ( ii = 0; ii < lVCount; ii++ )
      {
        // sm_CrvRemoveKnots( nlCurV[ii], dTol, nlCurV[ii], &nlStacks );
        N_CrvRemoveAllKnots(  nlCurV[ii], dTol, nlCurV[ii], &nlStacks );
        // N_CrvRemoveKnotsDerivConstraints( nlCurV[ii], dTol, constraints, der, nlCurV[ii], &nlStacks );
      }

    // Make curves compatible again.
    NL_SER( N_CrvsMakeCompatible( nlCurU, lUCount-1, &nlStacks ));
    NL_SER( N_CrvsMakeCompatible( nlCurV, lVCount-1, &nlStacks ));
} //cbi2
//cbi new: end.

  // Return args
  // if bCommonParameterization == false, then uParam & vParam will be NULL??
  if ( u_arg  != NULL ) { *u_arg  = uParam;  }
  if ( v_arg  != NULL ) { *v_arg  = vParam;  }
  if ( uu_arg != NULL ) { *uu_arg = uu; }
  if ( vv_arg != NULL ) { *vv_arg = vv; }

return SM_SUCCESS;

} // end PrepCurvesForSurfacing

/*******************************************************************//**
PURPOSE: This routine creates a Gordon surface interpolating a mesh of
    of NON-RATIONAL cross-sectional curves (i.e. u-curves & v-curves).

NOTES: 
    The input requirements for Gordon surfaces are fairly demanding.
    The input mesh curves must fully intersect, and the parameter values at
    the intersections must match fairly closely along each row and column
    of the input curves.  As a corollary to that, each set of curves must
    be parameterized in the same direction.

    Provided the mesh of curves truly intersect at every row,column point
    in the mesh, then this routine will try to modify the curves slightly
    to make them parametrically acceptable for the Gordon surface construction.
    However, you may have problems if the curves in the rows or columns
    are very different in properties.

    If your curve mesh does not produce an acceptable Gordon surface,
    you might try sampling the curves to create a mesh of points,
    and interpolating the points.

    If the input curves are rational, they will be approximated
    with nonrational curves (copies) for the surface creation.

IMPLEMENTATION NOTES ---
   All curves in each direction must have the same parameterization.
   We do that reparameterization in this routine: we map every curve
   parameter domain to the unit interval.  Instead of mapping the input
   curves in place, we leave those unchanged and (1) map the NLib curves
   that we create from them and pass to the NLib Gordon-surface routine,
   and (2) map the intersection parameters accordingly (because the
   intersections are done on the unmapped input curves).

***********************************************************************/
SmStatus SmBSplineSurface::CreateGordonSurface
  (const SmContext                 & crContext, // in
   const SmTArray<SmBSplineCurve*> & crUCurves, // in
   const SmTArray<SmBSplineCurve*> & crVCurves, // in
         SmBSplineSurface         *& rpNewSurf) // out
{
  NL_STACKS nlStacks;
  SmNLibStackHandler sSH(&nlStacks);

#define MAX_CURVES 640
  NL_CURVE *nlCurU[ MAX_CURVES ];
  NL_CURVE *nlCurV[ MAX_CURVES ];
#undef  MAX_CURVES

  NL_PARAMETER *u, *v;

  SmStatus eStat = SmBSplineSurface::PrepCurvesForSurfacing(
      crContext, // in
      crUCurves, // in
      crVCurves, // in
      nlCurU,    // out
      nlCurV,    // out
      nlStacks,
      -1.0,   // tol: let it calculate its own.
      TRUE,   // MakeNonRational: that's required.
      0,      // num cpts between profiles
      0,      // num cpts between rails
      TRUE,   // common param ranges in each direction
      TRUE,   // common parameterization in each direction
      &u, &v, // out
      NULL, NULL
  );

  if ( eStat != SM_SUCCESS )
    { return eStat; }

  ULONG lUCount = crUCurves.GetSize();
  ULONG lVCount = crVCurves.GetSize();

  // Set up for the NLib Gordon-surface call.
  NL_SURFACE sur;
  N_SrfInitArrays(&sur);


  NL_DEGREE udeg=3, vdeg=3;

  // the number of U crvs determines the degree to use for v
  if ((NL_DEGREE)lUCount <= vdeg)vdeg = (NL_DEGREE)lUCount - 1;
  if ((NL_DEGREE)lVCount <= udeg)udeg = (NL_DEGREE)lVCount - 1;

  NL_DEGREE utdeg = 3, vtdeg= 3;
  if ( utdeg > udeg ) { utdeg = udeg; }
  if ( vtdeg > vdeg ) { vtdeg = vdeg; }

  // Create Gordon patch using fixed degree 3 or less if not enough
  NL_SER(N_CreateGordonSrf( nlCurU, v, lUCount-1, nlCurV, u, lVCount-1,
                  udeg, vdeg, utdeg, vtdeg, NL_YES, &sur, &nlStacks, &nlStacks ));

  rpNewSurf = new (crContext) SmBSplineSurface((gw_SURFACE *)&sur) ;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe3 = FALSE;
  if (bDebugMe3) {
      sm_GraphicsLoop();
      smgfx_SetColor(0,0,1);
      rpNewSurf->DrawUV(1,1);
      rpNewSurf->Dump();
  }
#endif
  return SM_SUCCESS;

} // end SmBSplineSurface::CreateGordonSurface

/*******************************************************************//**
PURPOSE: This routine creates a surface from a boundary given as
     2 u-curves and 2 V-curves and an interior point sample.
NOTES: 
     The point sample needs to be dense enough to produce a valid
     solution.  There must not be large 'holes' of missing points.
     This routine sometimes produces disappointing results
    
***********************************************************************/
SmStatus SmBSplineSurface::CreateFromBoundary
(const SmContext & crContext,
   const SmTArray < SmPoint3d> & SamplePoints,    // sample interior points
   const SmTArray < SmBSplineCurve*> & crUCurves, // 2 u surface boundary curves
   const SmTArray < SmBSplineCurve*> & crVCurves, // 2 v surface boundary curves
   double dSTol,                                  // fit tolerance for surface
   SmBSplineSurface *& rpNewSurf)                 // output surface
{
    NL_STACKS nlStacks;
    SmNLibStackHandler sSH(&nlStacks);
    NL_SURFACE sur;
    N_SrfInitArrays(&sur);
    
    NL_CURVE  *nlCurU[2];
    NL_CURVE  *nlCurV[2];
    
    ULONG nP = SamplePoints.GetSize()-1;
    NL_POINT *P  = N_AllocPt1dArray(nP, &nlStacks);
    for (ULONG j = 0; j <= nP; j++)
    {
        P[j].x = SamplePoints[j].x; 
        P[j].y = SamplePoints[j].y;     
        P[j].z = SamplePoints[j].z;
    }
    
    ULONG i;
    for (i = 0; i < 2; i++)
    {
        SmBSplineCurve * pCurve = crUCurves[i];
        pCurve->GetOrCreateGwNurbPointer();
        SM_ASSERT(pCurve->GetGwNurbPointer() != NULL);
        
        SmObjDelete sClean;
        if (pCurve->IsRational())
        {
            SmBSplineCurve *pNewCurve = NULL;
            double cTol = SM_EFF_ZERO_SQRT * pCurve->ApproximateLength(pCurve->GetNaturalInterval(), 5);
            SER(pCurve->ApproximateRationalCurve(crContext, cTol, pNewCurve));
            sClean.SetObj(pNewCurve);
            pCurve = pNewCurve;
        }
        nlCurU[i] = sm_CreateNlibCurve(pCurve, nlStacks);
    }
    ULONG k;
    for (k = 0; k < 2; k++)
    {
        SmBSplineCurve * pCurve = crVCurves[k];
        pCurve->GetOrCreateGwNurbPointer();
        SM_ASSERT(pCurve->GetGwNurbPointer() != NULL);
        SmObjDelete sClean;
        if (pCurve->IsRational())
        {
            SmBSplineCurve *pNewCurve = NULL;
            double cTol = SM_EFF_ZERO_SQRT * pCurve->ApproximateLength(pCurve->GetNaturalInterval(), 5);
            SER(pCurve->ApproximateRationalCurve(crContext, cTol, pNewCurve));
            sClean.SetObj(pNewCurve);
            pCurve = pNewCurve;
        }
        nlCurV[k] = sm_CreateNlibCurve(pCurve, nlStacks);
    }
    
    // Make curves compatible
    NL_SER(N_CrvsMakeCompatible(nlCurU, 1, &nlStacks)); 
    NL_SER(N_CrvsMakeCompatible(nlCurV, 1, &nlStacks));
    NL_FLAG ftl;
    NL_SER(N_FitSrfApproxShape(P, nP, nlCurU, nlCurV, NULL,
                      NULL, dSTol, &sur, &ftl, &nlStacks, &nlStacks));
    rpNewSurf = new(crContext) SmBSplineSurface((gw_SURFACE *)&sur) ; 

    if (ftl == TRUE)
        return (SM_SUCCESS);

    return (SM_ERR_NOT_WITHIN_TOLERANCE);

} // end SmBSplineSurface::CreateFromBoundary

/*******************************************************************//**
PURPOSE:   create a non-rational arc-length parameterized copy of
              a 3d curve, and when asked, do the same for an optional
              UVTrimCurve.             

NOTES: when pCurve is NonRational, output New Curve and UVTrimCurve are copied from inputs
       When pCurve is Rational, pCurve is approximated and reparameterized in arc length
         and if pOptBaseSurface is given, dropped to build rpNewUVTrimCurve
***********************************************************************/
static SmStatus sm_CreateNonRationalCurve
  (const SmContext & crContext,        // in : context for new object construction
   SmCurve         * pCurve,           // in : curve to approximate
   SmBSplineCurve  * pOptUVTrimCurve,  // in : optional UVTrimCurve, copied to rpNewUVTimrCurve whenpCurve is nonRational
   SmSurface       * pOptBaseSurface,  // in : optional base surface, used for DropCurve when pCurve is Rational
   SmApproxTol3d     dApproxTol,       // in : max allowed deviation from NewCurve to 
   SmBSplineCurve *& rpNewCurve,       // out: non-rational approx of input pCurve
   SmBSplineCurve *& rpNewUVTrimCurve) // out: associated UVTrimCurve when
                                       //         (pCurve->IsRational && pOptBaseSurface)
                                       //      or (pCurve->NotRation && pOptUVTrimCurve)
                                       //      NULL otherwise
{
  // init output
  SmBSplineCurve *pNewCurve   = NULL;
  SmBSplineCurve *pNewUVTrimCurve = NULL;

  // locals
  SmBSplineCurve *pNurb = SM_CAST_PTR(SmBSplineCurve,pCurve);
  NER(pNurb);

  // for rational curves
  if (pNurb->IsRational()) 
    {
      // Build arc-length parameterized nonRational curve approximation
      SER(pNurb->ApproximateRationalCurve(crContext,dApproxTol,pNewCurve));
      SER(pNewCurve->ReparametrizeWithArcLength());

      // When given optional UVTrimCurve - build arc-lentgh nonRational UVTrimCurve approximation
      if(pOptBaseSurface)
        {
          SM_ASSERT(pOptBaseSurface != NULL) ;
          SmExtent1d sTrimIvl = pNewCurve->GetNaturalInterval();
          SmExtent2d sDomain  = pOptBaseSurface->GetNaturalUVDomain();
          double dMaxDist;
          double dDeviation;
          SmTArray<SmBSplineCurve*> sUVCurves;
          SER(pOptBaseSurface->DropCurve(crContext,sDomain,*pNewCurve,
                                         sTrimIvl,
                                         dApproxTol,
                                         dMaxDist,    // out: MaxDist(DropPt, 3dCurvePt), found by sampling, may be slightly less than actual max.
                                         dDeviation,  // out: MaxDist(DropPt->SrfNormalLine, 3dCurvePt), Quality Measure: expected to be close to 0.0
                                         sUVCurves));
          SM_ASSERT(sUVCurves.GetSize() == 1);
          pNewUVTrimCurve = sUVCurves[0];
        }
    } // end rational curve branch
  else // nonRational curve branch
    {
      // make a curve copy and optional UVTrimCurve copy
      pNewCurve = new(crContext) SmBSplineCurve(*pNurb);
      if (pOptUVTrimCurve) 
        {
          SM_ASSERT(pOptUVTrimCurve->IsRational() == FALSE) ;
          pNewUVTrimCurve = new(crContext) SmBSplineCurve(*pOptUVTrimCurve);
        }
    }

  // set output
  rpNewCurve       = pNewCurve;
  rpNewUVTrimCurve = pNewUVTrimCurve;

  // all done
  return SM_SUCCESS;

} // end sm_CreateNonRationalCurve 

/*******************************************************************//**
PURPOSE: Create a corner blend surface given this ordered and oriented
    set of 3D curves and a set of associated UV curves and Surfaces to define
    the cross tangency requirements. 
    
NOTES: 
    Returns SM_ERR when there are fewer than 4 curves in the input arrays.

    When bCreateBilinearPatch == TRUE, CurveCount must equal 4 and 
      crUVCurves and crSurfaces are optional. The patch will not be G1.

    When bCreateBilinearPath == FALSE, CurveCount must be equal to or greater than 4
      and an associated crUVCurve and crSurface is required for every curve boundary.
      Patches will be G1.
    
      Triangular patches can be made with three nonDegenerate curves and 
      a fourth degenerate curve. Pass in an associated UVTrimCurve and Surface
      for each of the nonDegenerate curves and pass in NULL values for the
      degenerate curve's associated UVTrimCurve and Surface. For all other
      cases, make sure all boundary curves are nonDegenerate and are
      associated with a valid UVTrimCruve/Surface pair. 
   
   Coons patches are made when the number of input curves is 3 or 4. 
   NSided patches are made when the input number of curves is greater than 4

   This function could be extended to mix G1 and C0 edges within a single patch.  
   If you need this functionality please make a request.

   Curve ordering: counterclockwise starting with the 'left' curve (u == u min).
   [0] 'left'    U min
   [1] 'bottom'  V min
   [2] 'right'   U max
   [3] 'top'     V max
   crCurveOrients: SM_OT_SAME means counterclockwise.

***********************************************************************/
SmStatus SmBSplineSurface::CreateCornerBlend
  (const SmContext                   & crContext,             // in : context for new object construction
   const SmTArray<SmCurve*>          & cr3DCurves,            // in : ordered array of 3d boundary curves
   const SmTArray<SmOrientType>      & crCurveOrients,        // in : SM_OT_SAME     = associated cr3DCurve parameterized in direction of Curve order
                                                              //      SM_OT_OPPOSITE = associated cr3DCurve parameterized opposite to direction of Curve order
   const SmTArray<SmBSplineCurve*>   & crUVCurves,            // in : associated UVTrimCurves
                                                              //      In the future we may allow NULL in this and the
                                                              //      surfaces array to indicate that there is no tangency
                                                              //      for this edge, for now they are required  
   const SmTArray<SmSurface*>        & crSurfaces,            // in : associated Surfaces
   double                              dThisApproxTol3d,      // in : max allowed 3D deviation from blendSurfaces to 3DCurves
   double                              dTangencyTolRadians,   // in : max allowed angle deviation along tangent edges
   SmTArray<SmSurface*>              & rBlendSurfaces,        // out: blended surface set 
   SmBoolean                           bCreateBilinearPatch)  // in : TRUE = Create bilinear patch (ignore crDerivCrvs)
                                                              //      FALSE= Create bicubic patch through curves and crossTangents
{
  // init output
  ULONG ii;
#ifdef SM_DEBUG_CODE
  ULONG jj;
#endif

  rBlendSurfaces.ReSet();

  // check state - must have 4 or more edges
  if (cr3DCurves.GetSize() < 4) SER(SM_ERR);

#ifdef SM_DEBUG_CODE
  SmEdge *pEdge = NULL ;
  SmFace *pFace = NULL ;
  for(ii=0;ii<cr3DCurves.GetSize();ii++)
    { if(pEdge == NULL) pEdge = cr3DCurves[ii] ? (SmEdge *)cr3DCurves[ii]->GetEdge() : NULL ; }
  for(ii=0;ii<crSurfaces.GetSize();ii++)
    { if(pFace == NULL) pFace = crSurfaces[ii] ? (SmFace *)crSurfaces[ii]->GetFace() : NULL ; }
  SmBrep *pBrep =   pFace ? pFace->GetBrep() 
                  : pEdge ? pEdge->GetBrep() : NULL ;

SmBoolean bDebugMe = FALSE ;
  // draw 
  if(bDebugMe)
    {
      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      for(ii=0;ii<crSurfaces.GetSize();ii++)
        { smgfx_SetLook(1,2, 0,1,1) ; if(crSurfaces[ii]) crSurfaces[ii]->DrawUV(8,8,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ; }
      for(ii=0;ii<cr3DCurves.GetSize();ii++)
        { smgfx_SetLook(3,4, 1,0,0) ; if(cr3DCurves[ii]) cr3DCurves[ii]->Draw(NULL, TRUE) ; sm_GraphicsLoop() ; }
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // locals
  SmTArray<SmBSplineCurve*>     sBoundaries;      // copies of all input cr3DCurves
  SmTArray<SmBSplineCurve*>     sUVCurves;        // copies of all crUVCurves
  SmTArray<SmBSplineCurve*>     sTanCurves;       // temporary CrossDerivative Curves defining patch G1 constraints
  SmObjsDelete<SmBSplineCurve*> sCleanBound(&sBoundaries);
  SmObjsDelete<SmBSplineCurve*> sCleanUV(&sUVCurves);
  SmObjsDelete<SmBSplineCurve*> sCleanTanC(&sTanCurves);

  SmTArray<double>              sBreaks;
  SmTArray<SmTangentField*>     sTanFields;
  SmObjsDelete<SmTangentField*> sCleanTanF(&sTanFields);
  ULONG                         l3DCurveCnt = cr3DCurves.GetSize() ;

  // check state - when bCreateBilinearPatch == FALSE, UVCurves are required for every cr3DCurve
  // check state - when bCreateBilinearPatch == FALSE, crSurfaces are required for every cr3DCurve
  //               except when the 4th curve in a 4 sided patch is degenerate
  if(bCreateBilinearPatch == FALSE)
    { 
      ULONG lNullCount = 0 ;

      // for every boundary curve
      for (ii=0; ii<l3DCurveCnt; ii++) 
        {
          // count the associated NULL UVTrimCurve and Surface inputs
          if(crUVCurves[ii] == NULL || crSurfaces[ii] == NULL)
            { lNullCount++ ; }

          // Check inputs for triangle, 4 sided and NSided cases
          if(   (l3DCurveCnt == 4 && lNullCount > 1)  // triangle corners require 3 g1 sides
             || (l3DCurveCnt >  4 && lNullCount > 0)) // require g1 on all NSided patche sides
            {
              NER(crUVCurves[ii]) ;
              NER(crSurfaces[ii]) ;
            }
        } // end iter every boundary curve to check
    } // end need to validate UVTrimCurve and Surface input check

  // convert all input edges from NURBs to NUBs
  // GWC: When there are rational curves, this step takes more time than the rest of the algorithm - is it needed?
  for (ii=0; ii<l3DCurveCnt; ii++) 
    {
      SmBSplineCurve *pBoundary = NULL;
      SmBSplineCurve *pPSCurve  = NULL;

      // copy NUBs and their UVTrimCurves, approx NURBs and drop them to make their UVTrimCurves
      SER(sm_CreateNonRationalCurve(crContext,              // in : context for new object construction                                       
                                    cr3DCurves[ii],         // in : curve to approximate                                                      
                                    crUVCurves[ii],         // in : optional UVTrimCurve, copied to rpNewUVTimrCurve whenpCurve is nonRational
                                    crSurfaces[ii],         // in : optional base surface, used for DropCurve when pCurve is Rational         
                                    dThisApproxTol3d/2.0,   // in : max allowed deviation from NewCurve to                                    
                                    pBoundary,              // out: non-rational approx of input pCurve                                       
                                    pPSCurve));             // out: associated UVTrimCurve when                                               
                                                            //         (pCurve->IsRational && pOptBaseSurface)                                
                                                            //      or (pCurve->NotRation && pOptUVTrimCurve)                                 
                                                            //      NULL otherwise  

      sBoundaries.Add(pBoundary);  // cleaned up 3d boundary curves
      sUVCurves.Add(pPSCurve);     // associated UVTrimCurves
    
    } // end iter all input curves setting parameterization and removing Rational representations

  // If we have 4 curves then we need to reverse opposite ones and make them compatable
  if (sBoundaries.GetSize() == 4) 
    {
      ULONG lLeft  = 0;
      ULONG lBot   = 1;
      ULONG lRight = 2;
      ULONG lTop   = 3;
      SmTArray<SmBSplineCurve*> sBounds, sTans;

      // for every boundary - set orientations
      for (ii=0; ii<4; ii++) 
        {
          // make bot & right orientation = Same
          //      top & left  orientation = Opposite
          if (   ((ii==lTop || ii==lLeft)  && crCurveOrients[ii] == SM_OT_SAME)
              || ((ii==lBot || ii==lRight) && crCurveOrients[ii] != SM_OT_SAME)) 
            {
              SmExtent1d sIvl = sBoundaries[ii]->GetNaturalInterval();
              SER(sBoundaries[ii]->ReverseParameterization(sIvl,sIvl));
              if (sUVCurves[ii]) 
                {
                  sIvl = sUVCurves[ii]->GetNaturalInterval();
                  SER(sUVCurves[ii]->ReverseParameterization(sIvl,sIvl));

                } // end UVCurve existence check
            } // end need to reverse parameterization check
        } // end iter every boundary setting orienations

      // make left/right curve pair compatible (union knot vectors)
      SmBSplineCurve           * apData[2];
      SmTArray<SmBSplineCurve*>  sCurves(2,apData);
      sCurves.Add(sBoundaries[lLeft]);
      sCurves.Add(sBoundaries[lRight]);

      // MakeCurvesCompatible() will end up normalizing the domains of the
      // curves to the unit interval.  Do that ahead of time so that we can
      // figure a tolerance for its calls to FixupKnotVector(), which happen
      // both before and after the normalization.
      SmExtent1d sUnitIvl( 0, 1 );
      sCurves[0]->EditParameterization( sUnitIvl );
      sCurves[1]->EditParameterization( sUnitIvl );

      // There's a hard-coded knot tolerance everywhere of 1e-8 ... use that,
      // increased a bit for safety.   [T1000C, bd, 21 Dec 05]
      double dKnotTol = 2.0e-8;
      SER(SmBSplineCurve::MakeCurvesCompatible(sCurves, dKnotTol ));
      
      // make top/bot curve pair compatible (union knot vectors)
      sCurves.ReSet();
      sCurves.Add(sBoundaries[lBot]);
      sCurves.Add(sBoundaries[lTop]);
      SER(SmBSplineCurve::MakeCurvesCompatible( sCurves, dKnotTol ));

      // when working with crossTangents (making g1 patches)
      if(bCreateBilinearPatch == FALSE)
        {
          // make sure all UVCurves have same parameter range as 3DCurve
          for (ii=0; ii<4; ii++) 
          {
              if (sUVCurves[ii]) 
                {
                  SmExtent1d sIvl = sBoundaries[ii]->GetNaturalInterval();
                  SER(sUVCurves[ii]->EditParameterization(sIvl));
                }
            }

          // for every boundary curve - Create Tangent Fields
          for (ii=0; ii<4; ii++) 
            {
              SmBSplineCurve * pTanCurve   = NULL;
              SmCurve        * pStartCurve =  (ii%2==1)
                                             ? sBoundaries[lLeft] // when ii == lTop or lBot
                                             : sBoundaries[lBot]; // when ii == lLeft or lRight
              SmCurve        * pEndCurve   =  (ii%2==1)
                                             ? sBoundaries[lRight] // when ii == lTop or lBot
                                             : sBoundaries[lTop];  // when ii == lLeft or lRight
              double           dNormlized  =  (ii>=2)
                                             ? 1.0     //when ii == lRight or lTop
                                             : 0.0 ;   //when ii == lLeft or lBot
              double           dStartParam = pStartCurve->GetNaturalInterval().Evaluate(dNormlized);
              double           dEndParam   = pEndCurve->GetNaturalInterval().Evaluate(dNormlized);

#ifdef SM_DEBUG_CODE
              // draw start/End curves, Start/Stop points and sBoundaries[ii]
              if (bDebugMe) 
                {
                  smgfx_Erase() ;
                  smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
                  for(jj=0;jj<crSurfaces.GetSize();jj++)
                    { smgfx_SetLook(1,2, 0,1,1) ; if(crSurfaces[jj]) crSurfaces[jj]->DrawUV(8,8,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ; }
                  for(jj=0;jj<cr3DCurves.GetSize();jj++)
                    { smgfx_SetLook(3,4, 1,0,0) ; if(cr3DCurves[jj]) cr3DCurves[jj]->Draw(NULL, TRUE) ; sm_GraphicsLoop() ; }

                  smgfx_SetLook(6,7, 1,0,0); pStartCurve->DrawAt(dStartParam,1); sm_GraphicsLoop();
                  smgfx_SetLook(3,4, 1,0,1); pStartCurve->Draw(); sm_GraphicsLoop();
                  smgfx_SetLook(6,7, 0,0,1); pEndCurve->DrawAt(dEndParam,1); sm_GraphicsLoop();
                  smgfx_SetLook(3,4, 0,0,1); pEndCurve->Draw(); sm_GraphicsLoop();
                  smgfx_SetLook(2,3, 0,1,1); sBoundaries[ii]->Draw(); sm_GraphicsLoop();
                  sm_GraphicsLoop();
                }
#endif // SM_DEBUG_CODE

              SmCurve * pBoundaryCurve = sBoundaries[ii];

              // GWC: does this sections need to be rebuilt
              // when given a neighbor surface - build CrossTangent function 
              if (crSurfaces[ii] != NULL) 
                {
                  // build temporary crossTangent curve function
                  SmTangentField * pTF = NULL;
                  SER(SmTangentField::Create(crContext,
                                             2.0*dThisApproxTol3d,
                                             *pBoundaryCurve, *sUVCurves[ii], *crSurfaces[ii],
                                             *pStartCurve, dStartParam, SM_OT_SAME,
                                             *pEndCurve,   dEndParam,   SM_OT_SAME,
                                             pTF));
                  SmObjDelete sDelete(pTF);

                  // build an approximate piecewise hermite curve to the crossTangent curve
                  SmTArray<double> sKnots;
                  pBoundaryCurve->GetKnots(sKnots);
                  double dMaxGap3d;
                  SER(pTF->ApproximateCurve(crContext,
                                            sKnots,
                                            dTangencyTolRadians,   
                                            dMaxGap3d,
                                            pTanCurve,
                                            TRUE,     // in : bOptCreateAnalytics      
                                            FALSE,    // in : bOptMatchParameterization
                                            FALSE)) ; // in : bJustCopyBSplines        

#ifdef SM_DEBUG_CODE
                  if (bDebugMe) 
                    {
                      smgfx_Erase() ;
                      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
                      for(jj=0;jj<crSurfaces.GetSize();jj++)
                        { smgfx_SetLook(1,2, 0,1,1) ; if(crSurfaces[jj]) crSurfaces[jj]->DrawUV(8,8,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ; }
                      for(jj=0;jj<cr3DCurves.GetSize();jj++)
                        { smgfx_SetLook(3,4, 1,0,0) ; if(cr3DCurves[jj]) cr3DCurves[jj]->Draw(NULL, TRUE) ; sm_GraphicsLoop() ; }

                      smgfx_SetLook(3,4, 1,0,1); pTF->Draw(); sm_GraphicsLoop();
                      smgfx_SetLook(6,7, 0,0,1); pEndCurve->DrawAt(dEndParam,1); sm_GraphicsLoop();
                      smgfx_SetLook(3,4, 0,0,1); pEndCurve->Draw(); sm_GraphicsLoop();
                      smgfx_SetLook(2,3, 0,1,1); sBoundaries[ii]->Draw(); sm_GraphicsLoop();
                      sm_GraphicsLoop();
                    }
#endif // SM_DEBUG_CODE
                } // end given NeighborSurface for this edge branch
              else // no neighbor Surface for this edge branch
                {
                  // This is to handle degenerate curve which does not 'lie' on a surface
                  // In other words, pBoundaryCurve need to be a point curve.

                  // check state - currently only degenerate curves allowed not to have a neighbor surface
                  if (!pBoundaryCurve->IsDegenerate()) SER(SM_ERR);

                  // get neighbor curve endPoint tangents
                  SmVector3d sPV1[2], sPV2[2];
                  SER(pStartCurve->Evaluate(dStartParam,1,TRUE,sPV1));
                  SER(pEndCurve->Evaluate(dEndParam,1,TRUE,sPV2));

                  //double dAngle;
                  //SER(sPV1[1].AngleBetween(sPV2[1],dAngle));
                  //if (dAngle < dTangencyTolRadians) {
                  //    SmVector3d sAvgVec = 0.5*(sPV1[1] + sPV2[1]);
                  //    sPV1[1] = sAvgVec;
                  //    sPV2[1] = sAvgVec;
                  //}

                  // Create degree 3 crossTangent Vector linear interpolation of crossTangent values 
                  SmPoint3d sData[4];
                  SmTArray<SmPoint3d> sCntrlPoly(4,sData);
                  sCntrlPoly.Add(sPV1[1]);
                  sCntrlPoly.Add(sPV1[1]);
                  sCntrlPoly.Add(sPV2[1]);
                  sCntrlPoly.Add(sPV2[1]);
                  double adKData[2];
                  SmTArray<double> sKnots(2,adKData,2);
                  sKnots[0] = 0.0; sKnots[1] = 1.0;
                  ULONG alKMData[2];
                  SmTArray<ULONG> sKnotMult(2,alKMData,2);
                  sKnotMult[0] = 4;
                  sKnotMult[1] = 4;
                  SER(SmBSplineCurve::CreateCanonical(crContext,3,3,
                                                      sCntrlPoly, SM_CF_UNSPECIFIED, sKnotMult, sKnots, 
                                                      SM_KT_UNSPECIFIED, NULL, NULL, pTanCurve));
                  NER(pTanCurve);
                  SmExtent1d sTrimIvl = pBoundaryCurve->GetNaturalInterval();
                  SER(pTanCurve->EditParameterization(sTrimIvl));

                  //SER(SmBSplineCurve::CreatePointCurve(crContext,sPV2[1],pTanCurve));   // parameterized from 0 to 1

#ifdef SM_DEBUG_CODE
                  if (bDebugMe) 
                    {
                      smgfx_Erase() ;
                      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
                      for(jj=0;jj<crSurfaces.GetSize();jj++)
                        { smgfx_SetLook(1,2, 0,1,1) ; if(crSurfaces[jj]) crSurfaces[jj]->DrawUV(8,8,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ; }
                      for(jj=0;jj<cr3DCurves.GetSize();jj++)
                        { smgfx_SetLook(3,4, 1,0,0) ; if(cr3DCurves[jj]) cr3DCurves[jj]->Draw(NULL, TRUE) ; sm_GraphicsLoop() ; }

                      smgfx_SetLook(1,3, 1,0,0); for (double dT=0.0; dT<=1.0; dT=dT+0.1) 
                                                   { SmVector3d sVec;
                                                     SER(pTanCurve->EvaluatePoint(sTrimIvl.Evaluate(dT),sVec));
                                                     sVec.Draw(&sPV1[0]);
                                                   }
                      sm_GraphicsLoop();
                    }
#endif // SM_DEBUG_CODE
                } // end no neighbor Surface for this edge branch

              // add crossTangent function to ordered array
              sTanCurves.Add(pTanCurve);
        
            } // end iter ii, every boundary edge creating crossTangent functions

          // create [left, right, bot, top] ordered crossTangent curve array
          sTans.Add(sTanCurves[lLeft]); 
          sTans.Add(sTanCurves[lRight]);
          sTans.Add(sTanCurves[lBot]);  
          sTans.Add(sTanCurves[lTop]);  

        } // end  working with crossTangents (making g1 patches) check

      // create [left, right, bot, top] ordered boundary curve array
      sBounds.Add(sBoundaries[lLeft]);  
      sBounds.Add(sBoundaries[lRight]); 
      sBounds.Add(sBoundaries[lBot]);   
      sBounds.Add(sBoundaries[lTop]);
      
      // let tol = SM_ZONE_TOL_3D * boundaryCurve->BoundingBox maxDimension
      SmExtent3d sBBox;
      for (ii=0; ii<4; ii++) 
        {
          SmExtent3d sCBox;
          SmExtent1d sIvl = sBoundaries[ii]->GetNaturalInterval();
          SER(sBoundaries[ii]->CalculateBoundingBox(sIvl,&sCBox));
          sBBox.Union(sCBox,sBBox);
        }
      double dTol = SM_ZONE_TOL_3D*sBBox.GetSize().GetMaxDimension();

      // build coons patch and save result
      SmBSplineSurface * pCoonsSurface = NULL;
      SER(SmBSplineSurface::CreateCoonsPatch(crContext,
                                             dTol, bCreateBilinearPatch,
                                             sBounds, sTans,
                                             pCoonsSurface));
      rBlendSurfaces.Add(pCoonsSurface);

      return SM_SUCCESS;

    } // end 4 sided patch check

  // N-sided blend
  SmTArray<double> sKnots;
  SmBoolean bNegateCurrNormal = FALSE ;
  SmBoolean bNegateTanCurves  = UNSURE ;

  // for every boundary - set orientations
  for (ii=0; ii<crCurveOrients.GetSize(); ii++) 
    {                                                          
      // make all curves SM_OT_SAME                            
      if(crCurveOrients[ii] == SM_OT_OPPOSITE)
        {
          SmExtent1d sIvl1 = sBoundaries[ii]->GetNaturalInterval();
          SER(sBoundaries[ii]->ReverseParameterization(sIvl1,sIvl1));
          
          if(sUVCurves[ii]) 
            {
              SmExtent1d sIvl2 = sUVCurves[ii]->GetNaturalInterval();
              SER(sUVCurves[ii]->ReverseParameterization(sIvl2,sIvl2));
            }
        }
    } // end iter every boundary setting orienations

  // Now Create CrossTangent Curves 
  for (ii=0; ii<sBoundaries.GetSize(); ii++) 
    {
      ULONG           lPrev = (ii==0) ? sBoundaries.GetSize()-1 : ii-1 ;
      ULONG           lNext = (ii==sBoundaries.GetSize()-1) ? 0 : ii+1;

      SmBSplineCurve *pCurr = sBoundaries[ii];
      SmBSplineCurve *pPrev = sBoundaries[lPrev];
      SmBSplineCurve *pNext = sBoundaries[lNext];
      
      SmBSplineCurve *pCurrUVCurve = sUVCurves[ii] ;
      SmBSplineCurve *pPrevUVCurve = sUVCurves[lPrev] ;
      SmBSplineCurve *pNextUVCurve = sUVCurves[lNext] ;

      SmSurface *pCurrSurface = crSurfaces[ii] ;    
      SmSurface *pPrevSurface = crSurfaces[lPrev] ;
      SmSurface *pNextSurface = crSurfaces[lNext] ;

      // locals - Previous boundary endPoint CrossTangent
      SmPoint3d  sPrevEndUV3d ;
      SmVector3d sPrevEndPV[2], sPrevEndNormal, sPrevEndNormal2 ;
      double     sPrevEndParam = pPrevUVCurve->GetNaturalInterval().GetMax() ; 
      SmVector3d sPrevEndCrossVec, sPrevEndCrossTang ;
      double     sPrevEndCurvature ;

      // locals - Current boundary startPoint CrossTangent
      SmPoint3d  sCurrStartUV3d ;
      SmVector3d sCurrStartPV[2], sCurrStartNormal, sCurrStartNormal2 ;
      double     sCurrStartParam = pCurrUVCurve->GetNaturalInterval().GetMin() ;
      SmVector3d sCurrStartCrossVec, sCurrStartCrossTang ;
      double     sCurrStartCurvature ;

      // locals - Current boundary endPoint CrossTangent
      SmPoint3d  sCurrEndUV3d ;
      SmVector3d sCurrEndPV[2], sCurrEndNormal, sCurrEndNormal2 ;
      double     sCurrEndParam = pCurrUVCurve->GetNaturalInterval().GetMax() ;
      SmVector3d sCurrEndCrossVec, sCurrEndCrossTang ;
      double     sCurrEndCurvature ;

      // locals - Next Boundary startPoint CrossTangent
      SmPoint3d  sNextStartUV3d ;
      SmVector3d sNextStartPV[2], sNextStartNormal, sNextStartNormal2 ;
      double     sNextStartParam = pNextUVCurve->GetNaturalInterval().GetMin() ;
      SmVector3d sNextStartCrossVec, sNextStartCrossTang ;
      double     sNextStartCurvature ;

      // 3d Boundary end Curve Points and tangents
      pPrev->Evaluate(sPrevEndParam,   1, TRUE, sPrevEndPV  ) ;
      pCurr->Evaluate(sCurrStartParam, 1, TRUE, sCurrStartPV) ;
      pCurr->Evaluate(sCurrEndParam,   1, TRUE, sCurrEndPV  ) ;
      pNext->Evaluate(sNextStartParam, 1, TRUE, sNextStartPV) ;

      // 2d Boundary end UV Points
      pPrevUVCurve->EvaluatePoint(sPrevEndParam,   sPrevEndUV3d) ;
      pCurrUVCurve->EvaluatePoint(sCurrStartParam, sCurrStartUV3d) ;
      pCurrUVCurve->EvaluatePoint(sCurrEndParam,   sCurrEndUV3d) ;
      pNextUVCurve->EvaluatePoint(sNextStartParam, sNextStartUV3d) ;
      
      // change UVPoint rep from 3d to 2d
      SmPoint2d  sPrevEndUV  (sPrevEndUV3d.x,   sPrevEndUV3d.y) ;
      SmPoint2d  sCurrStartUV(sCurrStartUV3d.x, sCurrStartUV3d.y) ;
      SmPoint2d  sCurrEndUV  (sCurrEndUV3d.x,   sCurrEndUV3d.y) ;
      SmPoint2d  sNextStartUV(sNextStartUV3d.x, sNextStartUV3d.y) ;
      
      // Surface Normals at boundary endPoints
      pPrevSurface->EvaluateNormal(sPrevEndUV,   TRUE, TRUE, sPrevEndNormal  ) ;
      pCurrSurface->EvaluateNormal(sCurrStartUV, TRUE, TRUE, sCurrStartNormal) ;
      pCurrSurface->EvaluateNormal(sCurrEndUV,   TRUE, TRUE, sCurrEndNormal  ) ;
      pNextSurface->EvaluateNormal(sNextStartUV, TRUE, TRUE, sNextStartNormal) ;
      
      // Curve CrossVecs at boundary endPoints (Curves that point out of the New Patch)
      // Tricky: we need binormals to the patch being built, but we don't
      //  know how the boundary surface normal's relate to that open patch.
      //  Those normals can either point in the same or opposite direction of 
      //  the patch to be built.
      //  We do know that: 
      //    1. all boundary curves are properly oriented to run around the new patch.
      //    2. and that boundary curves come with UVTrimCurves with the same orientation as their 3d curves.
      //  if we knew the orienation of the neighbor surface normals to the new patch normal
      //   then we can compute the binormal.  For now we'll make an assumption and see
      //    if that ever gets into trouble. The assumption comes in two parts.
      //    1. assume the first neighbor normal is the same as the upcoming patch's normal.
      //      That means on that surface the right hand rule points into the new patch.
      //      Then compare each corner point's pair of normals to classify the sequence of neighbor normals.
      //    At this point all binormal directions will be consistent with one another but
      //    if the original assumption is wrong, all those directions will be negated.
      //    2. If any of the neighbor curves are on the boundary of a surface we will assume
      //       that surface is outside the new patch.  When we find this surface
      //       we'll be able to adjust for incorrect first assumptions.

      // compare the neighbor and current surface normal directions
      double dPrevDotCurrNormal = sPrevEndNormal.Dot(sCurrStartNormal) ; 
      double dNextDotCurrNormal = sNextStartNormal.Dot(sCurrEndNormal) ; 

      // these are expected to be parallel
      SM_ASSERT_MSG_BREAK(   sPrevEndNormal.IsParallelTo(sCurrStartNormal, 5.0)
                          && sNextStartNormal.IsParallelTo(sCurrEndNormal, 5.0)
                          && (smos_Fabs(dPrevDotCurrNormal) > SM_EFF_ZERO)
                          && (smos_Fabs(dNextDotCurrNormal) > SM_EFF_ZERO),
                          _T("CreateNSidedPatch: Given neighbor patches that are not tangent to another at the patch corners")) ;

      // remember the neighbor to curr normal relationships
      SmBoolean bPrevNormalSame = dPrevDotCurrNormal > 0.0 ;
      SmBoolean bNextNormalSame = dNextDotCurrNormal > 0.0 ;

      // flip surface normals as needed
      if (   ( bNegateCurrNormal &&  bPrevNormalSame)
          || (!bNegateCurrNormal && !bPrevNormalSame)) { sPrevEndNormal    = -sPrevEndNormal    ; }
      if (     bNegateCurrNormal)                      { sCurrStartNormal  = -sCurrStartNormal  ; }
      if (     bNegateCurrNormal)                      { sCurrEndNormal    = -sCurrEndNormal    ; }
      if (   ( bNegateCurrNormal &&  bNextNormalSame)
          || (!bNegateCurrNormal && !bNextNormalSame)) { sNextStartNormal  = -sNextStartNormal  ; }

      // get ready for next iteration
      bNegateCurrNormal =   ( bNegateCurrNormal &&  bNextNormalSame)
                         || (!bNegateCurrNormal && !bNextNormalSame) ;

      // Let CrossVec be the direction out of the new patch 
      //   (if the first neighbor surface was negated these will be pointing inward - we'll have to catch that later) 
      sPrevEndCrossVec   = sPrevEndNormal   * sPrevEndPV[1] ;
      sCurrStartCrossVec = sCurrStartNormal * sCurrStartPV[1] ;
      sCurrEndCrossVec   = sCurrEndNormal   * sCurrEndPV[1] ;
      sNextStartCrossVec = sNextStartNormal * sNextStartPV[1] ;

      // see if the first assumption about surface normals can be corrected by a boundary surface case
      if(bNegateTanCurves == UNSURE)
        {
          // classify the ends of the curve against the surface UVDomain
          SmExtent2d sCurrUVDomain = crSurfaces[ii]->GetNaturalUVDomain() ;
          SmPoint2d  sTolUV        = sCurrUVDomain.GetSize() / 1000 ;
          double     dTol          = smos_Min(sTolUV.x, sTolUV.y) ;
          ULONG      sStartBndries = sCurrUVDomain.GetPoint2dBoundaries(sCurrStartUV, dTol) ; // orof:[SM_SS_UMIN SM_SS_VMIN SM_SS_UMAX SM_SS_VMAX]
          ULONG      sEndBndries   = sCurrUVDomain.GetPoint2dBoundaries(sCurrEndUV, dTol) ;
          ULONG      sBndries      = sStartBndries & sEndBndries ;

          // when pCurr starts and stop on a common boundary - check orientation
          if(sBndries)
             {
               // When the curve is a 
               //   - minDomain boundary, the surface's cross tangent vector points out of the patch
               //   - maxDomain boundary, the surface's cross tangent vector points into the patch

               // Surface 1st derivs at CurveStart
               SmVector3d sSrfPt, sSrfDu, sSrfDv ;
               pCurrSurface->Evaluate1stDerivatives(sCurrStartUV, TRUE, TRUE, sSrfPt, sSrfDu, sSrfDv) ;

               // pick the approriate vector and make it point out of the patch
               SmVector3d sOutward =   (sBndries & SM_SS_UMIN)   ?  sSrfDu
                                     : (sBndries & SM_SS_VMIN)   ?  sSrfDv
                                     : (sBndries & SM_SS_UMAX)   ? -sSrfDu
                                     /*(sBndries & SM_SS_VMAX)*/ : -sSrfDv ;

               // remember if the CurrStart outward and crossVec are parallel or antiparallel
               // The vector fields will be negated later on
               double dCrossVecDotOutward = sCurrStartCrossVec.Dot(sOutward) ;
               bNegateTanCurves = (dCrossVecDotOutward < 0.0) ;
                
             } // end found a surface bounding case check
         } // end need to set orientation check

      // Surface CrossTangents at boundary endPoints
      pPrevSurface->EvaluateNormalSection(sPrevEndUV,   TRUE, TRUE, sPrevEndCrossVec,   sPrevEndCrossTang,   sPrevEndCurvature,   sPrevEndNormal2) ;
      pCurrSurface->EvaluateNormalSection(sCurrStartUV, TRUE, TRUE, sCurrStartCrossVec, sCurrStartCrossTang, sCurrStartCurvature, sCurrStartNormal2) ;
      pCurrSurface->EvaluateNormalSection(sCurrEndUV,   TRUE, TRUE, sCurrEndCrossVec,   sCurrEndCrossTang,   sCurrEndCurvature,   sCurrEndNormal2) ;
      pNextSurface->EvaluateNormalSection(sNextStartUV, TRUE, TRUE, sNextStartCrossVec, sNextStartCrossTang, sNextStartCurvature, sNextStartNormal2) ;

      // check boundary curve gap sizes
      double dDistPrev = sPrevEndPV[0].DistanceBetween(sCurrStartPV[0]);
      double dDistNext = sCurrEndPV[0].DistanceBetween(sNextStartPV[0]);
      if(   dDistPrev > dThisApproxTol3d
         || dDistNext > dThisApproxTol3d) 
        { SER(SM_ERR) ; } // Ends don't match

      // get the desired tangentField endTangent vecs
      SmVector3d sStartTF = (sPrevEndCrossTang + sCurrStartCrossTang) / 2.0 ;
      SmVector3d sEndTF   = (sCurrEndCrossTang + sNextStartCrossTang) / 2.0 ;

      // build a tangent field (derived from SmCrvOnSurf) for this boundary from its surface and given UVTrimCurve 
      SmTangentField * pTF = new(crContext) SmTangentField          
                                              (*pCurrUVCurve,       // in : TrimCurve                                          
                                               *pCurrSurface,       // in : assocaited Surface                                 
                                                SM_INTERPOLATE_DIR, // in : oneof: SM_CONSTANT_DIR,                            
                                                                    //             SM_INTERPOLATE_DIR, <== only supported value
                                                                    //             SM_PERPENDICULAR_DIR                        
                                                sStartTF,           // in : TangentStartValue                                  
                                                sEndTF);            // in : TangentEndValue                                    
      sTanFields.Add(pTF);                                          // in : opt subDomain of projection surface                

      // use pCurr knot vector
      pCurr->GetKnots(sKnots);
      double dAchieved;

      // build a curve approximation to the tangent field using pCurr knot vector
      SmBSplineCurve *pTanCurve = NULL ;
      SER(pTF->ApproximateCurve(crContext,
                                sKnots,
                                dTangencyTolRadians,   
                                dAchieved,
                                pTanCurve,
                                TRUE,     // in : bOptCreateAnalytics      
                                FALSE,    // in : bOptMatchParameterization
                                FALSE)) ; // in : bJustCopyBSplines        
      sTanCurves.Add(pTanCurve);

#ifdef SM_DEBUG_CODE
      // draw 
      if(bDebugMe)
        {
          SM_DUMP_AND_ASSERT_VALID(pPrev) ;
          SM_DUMP_AND_ASSERT_VALID(pCurr) ;
          SM_DUMP_AND_ASSERT_VALID(pNext) ;
                  
          SM_DUMP_AND_ASSERT_VALID(pPrevUVCurve) ;
          SM_DUMP_AND_ASSERT_VALID(pCurrUVCurve) ;
          SM_DUMP_AND_ASSERT_VALID(pNextUVCurve) ;

          pFace = (SmFace *)pCurrSurface->GetFace() ;
          pEdge = (SmEdge *)pCurr->GetEdge() ;
          pBrep =   pFace ? pFace->GetBrep() : pEdge ? pEdge->GetBrep() : NULL ;

          SmCrvOnSurf sPrevCOS(*pPrevUVCurve, *pPrevSurface) ;
          SmCrvOnSurf sCurrCOS(*pCurrUVCurve, *pCurrSurface) ;
          SmCrvOnSurf sNextCOS(*pNextUVCurve, *pNextSurface) ;

          smgfx_Erase() ;
          smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;

          // surfaces
          smgfx_SetLook(1,2, 0,1,1) ; if(pPrevSurface) pPrevSurface->DrawUV(8,8,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 0,1,1) ; if(pCurrSurface) pCurrSurface->DrawUV(8,8,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 0,1,1) ; if(pNextSurface) pNextSurface->DrawUV(8,8,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ;
          
          // UVtrimCurves                                                                                  
          smgfx_SetLook(2,3, 0,1,0) ; if(pPrevSurface) pPrevSurface->DrawUVCurve(*pPrevUVCurve) ; sm_GraphicsLoop() ;
          smgfx_SetLook(2,3, 1,0,0) ; if(pCurrSurface) pCurrSurface->DrawUVCurve(*pCurrUVCurve) ; sm_GraphicsLoop() ;
          smgfx_SetLook(2,3, 0,0,1) ; if(pNextSurface) pNextSurface->DrawUVCurve(*pNextUVCurve) ; sm_GraphicsLoop() ;

          // UVtrimCurves parameterized                                                                                 
          smgfx_SetLook(2,3, 0,1,0) ; if(pPrevSurface) sPrevCOS.DrawParams() ; sm_GraphicsLoop() ;
          smgfx_SetLook(2,3, 1,0,0) ; if(pCurrSurface) sCurrCOS.DrawParams() ; sm_GraphicsLoop() ;
          smgfx_SetLook(2,3, 0,0,1) ; if(pNextSurface) sNextCOS.DrawParams() ; sm_GraphicsLoop() ;

          // UVTrimCurve/3DCurve gaps
          smgfx_SetLook(2,3, 0,1,0) ; if(pPrevSurface) pPrevSurface->DrawInspectUVTrimCurve(*pPrevUVCurve, *pPrev) ; sm_GraphicsLoop() ;
          smgfx_SetLook(2,3, 1,0,0) ; if(pCurrSurface) pCurrSurface->DrawInspectUVTrimCurve(*pCurrUVCurve, *pCurr) ; sm_GraphicsLoop() ;
          smgfx_SetLook(2,3, 0,0,1) ; if(pNextSurface) pNextSurface->DrawInspectUVTrimCurve(*pNextUVCurve, *pNext) ; sm_GraphicsLoop() ;

          // Curr Start/End VectorField (G1 crossDer direction) values
          smgfx_SetLook(1,5, 1,.5,0); sStartTF.Draw(&sCurrStartPV[0]) ;   sm_GraphicsLoop() ;
          smgfx_SetLook(1,5, 1,.5,0); sEndTF  .Draw(&sCurrEndPV[0]  ) ; sm_GraphicsLoop() ;

          // the vector field
          smgfx_SetLook(1,2, 0,1,0) ; if(pTF)   pTF->Draw() ; sm_GraphicsLoop() ;
          sm_GraphicsLoop() ;

          // intermediate values

          // CurveEnds pos/tan
          smgfx_SetLook(1,5, 1,0,1) ; sPrevEndPV[0]  .Draw() ; sPrevEndPV[1]  .Draw(&sPrevEndPV[0]  ) ;   sm_GraphicsLoop() ;
          smgfx_SetLook(1,5, 1,0,0) ; sCurrStartPV[0].Draw() ; sCurrStartPV[1].Draw(&sCurrStartPV[0]) ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,5, 1,0,0) ; sCurrEndPV[0]  .Draw() ; sCurrEndPV[1]  .Draw(&sCurrEndPV[0]  ) ;   sm_GraphicsLoop() ;
          smgfx_SetLook(1,5, 1,0,1) ; sNextStartPV[0].Draw() ; sNextStartPV[1].Draw(&sNextStartPV[0]) ; sm_GraphicsLoop() ;

          // CurveEnds SurfNormals - These are the processed surface normals (expected to be aligned)
          smgfx_SetLook(1,5, 1,0,1) ; sPrevEndNormal  .Draw(&sPrevEndPV[0]  ) ;   sm_GraphicsLoop() ;
          smgfx_SetLook(1,5, 1,0,0) ; sCurrStartNormal.Draw(&sCurrStartPV[0]) ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,5, 1,0,0) ; sCurrEndNormal  .Draw(&sCurrEndPV[0]  ) ;   sm_GraphicsLoop() ;
          smgfx_SetLook(1,5, 1,0,1) ; sNextStartNormal.Draw(&sNextStartPV[0]) ; sm_GraphicsLoop() ;

          // CurveEnds CrossVecs = Direction which projects to the tangent of a curve on             
          //                       the surface at which the evaluation for crossTangent and CrossCurvature is being done. Non-unit is OK.
          smgfx_SetLook(1,5, 0,0,1) ; sPrevEndCrossVec  .Draw(&sPrevEndPV[0]  ) ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,5, 0,0,1) ; sCurrStartCrossVec.Draw(&sCurrStartPV[0]) ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,5, 0,0,1) ; sCurrEndCrossVec  .Draw(&sCurrEndPV[0]  ) ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,5, 0,0,1) ; sNextStartCrossVec.Draw(&sNextStartPV[0]) ; sm_GraphicsLoop() ;

          // CurveEnds CrossTangs = Unit-Vector in direction of crDirection projected into tangent plane.
          //                        (when CrossVec is tangent to surface, CrossVec and CrossTang will have the same direction)
          smgfx_SetLook(2,5, 0,1,0) ; sPrevEndCrossTang  .Draw(&sPrevEndPV[0]  ) ;   sm_GraphicsLoop() ;
          smgfx_SetLook(2,5, 0,1,0) ; sCurrStartCrossTang.Draw(&sCurrStartPV[0]) ; sm_GraphicsLoop() ;
          smgfx_SetLook(2,5, 0,1,0) ; sCurrEndCrossTang  .Draw(&sCurrEndPV[0]  ) ;   sm_GraphicsLoop() ;
          smgfx_SetLook(2,5, 0,1,0) ; sNextStartCrossTang.Draw(&sNextStartPV[0]) ; sm_GraphicsLoop() ;

          // CurveEnds SurfNormals computed a 2nd time by EvaluateNormalSection - These are the raw surface normals (randomly aligned)
          smgfx_SetLook(3,5, 1,0,1) ; sPrevEndNormal2  .Draw(&sPrevEndPV[0]  ) ;   sm_GraphicsLoop() ;
          smgfx_SetLook(3,5, 1,0,0) ; sCurrStartNormal2.Draw(&sCurrStartPV[0]) ; sm_GraphicsLoop() ;
          smgfx_SetLook(3,5, 1,0,0) ; sCurrEndNormal2  .Draw(&sCurrEndPV[0]  ) ;   sm_GraphicsLoop() ;
          smgfx_SetLook(3,5, 1,0,1) ; sNextStartNormal2.Draw(&sNextStartPV[0]) ; sm_GraphicsLoop() ;
        }
#endif // SM_DEBUG_CODE

    } // end iter every boundary curve creating a TangentField

  // When we've figured out that the crossTangent vectors need to be negated
  if(bNegateTanCurves == TRUE)
    {
      // negate every sTanCurve 
      for(ii=0;ii<sTanCurves.GetSize();ii++)
        {
          SmBSplineCurve *pTanCurve = sTanCurves[ii] ;
          pTanCurve->Scale(-1.0) ;

        } // end iter every sTanCurves
    } // end need to negate sTanCurves check

  if(bNegateTanCurves == UNSURE)
    {
      WARN(_T("SmBSplineSurface::CreateCornerBlend found an ambiguous orientation case - additional constraints on the inputs may have to be added")) ;
    }

#ifdef SM_DEBUG_CODE
  // draw input to upcoming CreateNSidedPatch call
  if(bDebugMe)
    {
      static constexpr ULONG lSmpCnt = 25 ;
      SmTArray<SmPoint3d> sPnts(lSmpCnt,0,lSmpCnt), sVecs(lSmpCnt,0,lSmpCnt) ;
      SmExtent1d sIvl ;
      double dParam ;

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;

      // for every boundary
      for(ii=0;ii<sBoundaries.GetSize();ii++)
        {
          smgfx_SetLook(1,2) ; // changes color
          smgfx_SetLineWidth(3) ; if(sBoundaries[ii]) sBoundaries[ii]->Draw() ; sm_GraphicsLoop() ;
          smgfx_SetLineWidth(1) ; if(sBoundaries[ii]) sBoundaries[ii]->DrawParams() ; sm_GraphicsLoop() ;
          smgfx_SetLineWidth(1) ; if(crSurfaces[ii]) crSurfaces[ii]->DrawUV(8,8,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ;
          sIvl = sBoundaries[ii]->GetNaturalInterval() ;
          for(jj=0;jj<lSmpCnt;jj++) { dParam = (double)jj / (double)(lSmpCnt-1) ;
                                      sBoundaries[ii]->EvaluatePoint(sIvl.Evaluate(dParam), sPnts[ii]) ;
                                      sTanCurves[ii]->EvaluatePoint(sIvl.Evaluate(dParam), sVecs[ii]) ;
                                    }
          smgfx_SetLineWidth(1) ; smgfx_DrawComb((double *)sVecs.GetDataArray(), (double *)sPnts.GetDataArray(), lSmpCnt, 2.0) ;
        }
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // pass the call along to create the NSidedPatch
  sCleanUV.Clear();
  SER(SmBSplineSurface::CreateNSidedPatch(crContext,
                                          sBoundaries,
                                          sTanCurves,
                                          dTangencyTolRadians,  // CreateNSidedPatch specifies tolerance in degrees
                                          rBlendSurfaces));

  return SM_SUCCESS;

} // end SmBSplineSurface::CreateCornerBlend

/* ---------------------------------------------------------------------


   DESCRIPTION:

     This advanced surface construction routine creates a bicubic Coons
     surface interpolating four NON-RATIONAL boundary curves and cross-
     boundary derivatives.  The boundary curves are assumed to intersect
     at their respective end points.  The boundary curves and the corresponding
     cross-boundary derivative curves ("CBD's") are assumed to be compatible
     in the B-Spline sense.  Also, the directions of the CBD's must match the
     derivatives of the corresponding boundary curves at their ends: for example,
     the value of the bottom CBD curve at its beginning must be parallel to
     the first derivative of the left boundary curve at its beginning.  If
     these conditions are not met, then the resulting surface might not match
     the CBD's, i.e., it might not meet adjacent surfaces smoothly.  Note
     however that the magnitudes of those derivatives need not match: this
     routine will scale the CBD curves in place to match the derivatives of
     the boundary curves where they meet.
     If the output surface is initialized to NULL, memory to store new
     control points and knots is allocated.  A typical calling example is:

       CURVE    curL, derL, curR, derR, curB, derB, curT, derT;
       SURFACE  sur;
       STACKS   SC, SS;
       ...
       (get boundary curves and derivatives);
       ...
       N_SrfInitArrays(&sur);
       N_CreateCoonsBoundaryCrvs(&curL,&derL,&curR,&derR,&curB,&derB,&curT,&derT,
                &sur,&SC,&SS);

     If memory is  available, sur  is not  initialized  and the routine
     assumes that memory allocation  has been done. However, it  checks
     for the proper amount by looking  at the highest  indexes in sur's
     knot  vector and  polygon  objects.  ALL INPUT DATA MUST BE ON THE
     SAME STACK  'SC'. OPPOSITE INPUT  CURVES AND  DERIVATIVES  MUST BE
     COMPATIBLE IN  THE  B-SPLINE  SENSE. IF  THEY ARE NOT, THE ROUTINE
     MAKES THEM COMPATIBLE, I.E. THE INPUT DATA MAY BE DESTROYED.


   ACCESS:

     curL  , in/out ,  Left boundary
     derL  , in/out ,  Cross-boundary derivative across curL
     curR  , in/out ,  Right boundary
     derR  , in/out ,  Cross-boundary derivative across curR
     curB  , in/out ,  Bottom boundary
     derB  , in/out ,  Cross-boundary derivative across curB
     curT  , in/out ,  Top boundary
     derT  , in/out ,  Cross-boundary derivative across curT
     sur   , output ,  Bicubic Coons surface
     SC    , input  ,  curs' and ders' memory stack
     SS    , input  ,  sur's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NERR

   ------------------------------------------------------------------ */
NL_PRIVATE const NL_STRING  rname = NULL;

static NL_FLAG  sm_CreateCoonsSrf
  (NL_REAL MTOL,
   NL_CURVE *curL,
   NL_CURVE *derL,
   NL_CURVE *curR,
   NL_CURVE *derR,
   NL_CURVE *curB,
   NL_CURVE *derB,
   NL_CURVE *curT,
   NL_CURVE *derT,
   NL_SURFACE *sur,
   NL_STACKS *SC,
   NL_STACKS *SS,
   ULONG & lWarning )

{
  NL_FLAG     error = NL_NO;

  NL_INDEX    i, j, n, m, r, s;

  NL_DEGREE   p, q;

  NL_REAL     *U, *V, *UA, *VA, one_3, one_9, mag, da, db;

  NL_POINT     CD1[2], CD2[2], T00, T01, T10, T11, A, B, sTanVal1, sTanVal2;

  NL_CPOINT   **Pw, **Uw, **Vw, **Tw, *Aw, T00w, T10w, T01w, T11w;

  NL_CURVE    **curLR, **curBT;

  NL_SURFACE  **surA, surU, surV, surT;

  NL_STACKS   SL;

  NL_REAL     dTwistTol = MTOL*1.0e4;

  ULONG    lMisMatch;

  /* Start NURBS */

  lWarning = 0;

  N_InitNurbs(&SL);


  /* Check rationality and initialize constants */

  if ( N_IsCrvRat(curL) )  { NL_ERROR(NL_INP_ERR); }
  if ( N_IsCrvRat(curR) )  { NL_ERROR(NL_INP_ERR); }
  if ( N_IsCrvRat(curB) )  { NL_ERROR(NL_INP_ERR); }
  if ( N_IsCrvRat(curT) )  { NL_ERROR(NL_INP_ERR); }
  if ( N_IsCrvRat(derL) )  { NL_ERROR(NL_INP_ERR); }
  if ( N_IsCrvRat(derR) )  { NL_ERROR(NL_INP_ERR); }
  if ( N_IsCrvRat(derB) )  { NL_ERROR(NL_INP_ERR); }
  if ( N_IsCrvRat(derT) )  { NL_ERROR(NL_INP_ERR); }


  one_3 = 1.0 / 3.0;
  one_9 = 1.0 / 9.0;


  /* Make input data compatible in the B-spline sense */

  curLR = N_AllocArrayCrvPtrs(3,SC);  if( curLR EQ NULL ) { NL_QUIT; }
  curBT = N_AllocArrayCrvPtrs(3,SC);  if( curBT EQ NULL ) { NL_QUIT; }

  curLR[0] = curL;
  curLR[1] = curR;
  curLR[2] = derL;
  curLR[3] = derR;

  curBT[0] = curB;
  curBT[1] = curT;
  curBT[2] = derB;
  curBT[3] = derT;

  if( NOT N_CrvsAreCombatible( curLR, 3 ) )
  {
    error = N_CrvsMakeCompatible(curLR,3,SC);  if( error EQ NL_YES ) { NL_OUT; }
  }

  if( NOT N_CrvsAreCombatible( curBT, 3 ) )
  {
    error = N_CrvsMakeCompatible(curBT,3,SC);  if( error EQ NL_YES ) { NL_OUT; }
  }


  /*
   * Check twist compatibility: cross-boundary derivative curves
   * must match the first derivatives of the boundary curves
   * at the corners.  If they don't, the resulting surface might
   * not match the input tangent fields smoothly.  In that case,
   * we correct their magnitudes.  If that correction does not work,
   * we set lWarning to 1.
   */

  N_CrvGetKnots( curBT[0], &r, &U );
  N_CrvGetKnots( curLR[0], &s, &V );

  double dScale0, dScale1;


  /* Check curLR[2]: derL. */
  error = N_CrvEval  ( curLR[2], V[0], NL_LEFT,  &sTanVal1  );  if( error EQ NL_YES ) { NL_OUT; }
  error = N_CrvEval  ( curLR[2], V[s], NL_LEFT,  &sTanVal2  );  if( error EQ NL_YES ) { NL_OUT; }
  error = N_CrvDerivs( curBT[0], U[0], NL_LEFT, 1, CD1      );  if( error EQ NL_YES ) { NL_OUT; }
  error = N_CrvDerivs( curBT[1], U[0], NL_LEFT, 1, CD2      );  if( error EQ NL_YES ) { NL_OUT; }

#if 0
  // Check angles of corner vectors.
  double dChkAng;
  static double dAngLim = SM_DEG2RAD( 1.0 );

  N_VectorsAngle( sTanVal1, CD1[1], &dChkAng );
  if ( dChkAng > dAngLim )
    { dChkAng *= -1; } // Just for a breakpoint.
  N_VectorsAngle( sTanVal2, CD2[1], &dChkAng );
  if ( dChkAng > dAngLim )
    { dChkAng *= -1; }
#endif

  lMisMatch = 0;
  N_VectorDiff( sTanVal1, CD1[1], &A );
  N_VectorDiff( sTanVal2, CD2[1], &B );
  N_VectorMagnitude( A, &da );  if( da GT MTOL ) { lMisMatch = 1; }
  N_VectorMagnitude( B, &db );  if( db GT MTOL ) { lMisMatch = 1; }

  if ( lMisMatch > 0 )
  {
      // Scale the CBD curve by the start and end magnitude ratios.
      N_VectorMagnitude( sTanVal1, &da );
      N_VectorMagnitude( CD1[1],   &db );
      dScale0 = db / da;
      N_VectorMagnitude( sTanVal2, &da );
      N_VectorMagnitude( CD2[1],   &db );
      dScale1 = db / da;
      N_LinearMultiplyCrv( dScale0, dScale1, curLR[2] );
      lMisMatch = 0;

      // Recheck them.
      error = N_CrvEval  ( curLR[2], V[0], NL_LEFT,  &sTanVal1  );  if( error EQ NL_YES ) { NL_OUT; }
      error = N_CrvEval  ( curLR[2], V[s], NL_LEFT,  &sTanVal2  );  if( error EQ NL_YES ) { NL_OUT; }
      N_VectorDiff( sTanVal1, CD1[1], &A );
      N_VectorDiff( sTanVal2, CD2[1], &B );
      N_VectorMagnitude( A, &da );  if( da GT MTOL ) { lMisMatch = 1; }
      N_VectorMagnitude( B, &db );  if( db GT MTOL ) { lMisMatch = 1; }
      if ( lMisMatch > 0 )
        { lWarning = 1; }
  }


  /* Check curLR[3]: derR. */
  error = N_CrvEval  ( curLR[3], V[0], NL_LEFT,  &sTanVal1  );  if( error EQ NL_YES ) { NL_OUT; }
  error = N_CrvEval  ( curLR[3], V[s], NL_LEFT,  &sTanVal2  );  if( error EQ NL_YES ) { NL_OUT; }
  error = N_CrvDerivs( curBT[0], U[r], NL_LEFT, 1, CD1      );  if( error EQ NL_YES ) { NL_OUT; }
  error = N_CrvDerivs( curBT[1], U[r], NL_LEFT, 1, CD2      );  if( error EQ NL_YES ) { NL_OUT; }

  lMisMatch = 0;
  N_VectorDiff( sTanVal1, CD1[1], &A );
  N_VectorDiff( sTanVal2, CD2[1], &B );
  N_VectorMagnitude( A, &da );  if( da GT MTOL ) { lMisMatch = 1; }
  N_VectorMagnitude( B, &db );  if( db GT MTOL ) { lMisMatch = 1; }

  if ( lMisMatch > 0 )
  {
      N_VectorMagnitude( sTanVal1, &da );
      N_VectorMagnitude( CD1[1],   &db );
      dScale0 = db / da;
      N_VectorMagnitude( sTanVal2, &da );
      N_VectorMagnitude( CD2[1],   &db );
      dScale1 = db / da;
      N_LinearMultiplyCrv( dScale0, dScale1, curLR[3] );
      lMisMatch = 0;

      // Recheck them.
      error = N_CrvEval  ( curLR[3], V[0], NL_LEFT,  &sTanVal1  );  if( error EQ NL_YES ) { NL_OUT; }
      error = N_CrvEval  ( curLR[3], V[s], NL_LEFT,  &sTanVal2  );  if( error EQ NL_YES ) { NL_OUT; }
      N_VectorDiff( sTanVal1, CD1[1], &A );
      N_VectorDiff( sTanVal2, CD2[1], &B );
      N_VectorMagnitude( A, &da );  if( da GT MTOL ) { lMisMatch = 1; }
      N_VectorMagnitude( B, &db );  if( db GT MTOL ) { lMisMatch = 1; }
      if ( lMisMatch > 0 )
        { lWarning = 1; }
  }



  /* Check curBT[2]: derB. */
  error = N_CrvEval  ( curBT[2], U[0], NL_LEFT,  &sTanVal1  );  if( error EQ NL_YES ) { NL_OUT; }
  error = N_CrvEval  ( curBT[2], U[r], NL_LEFT,  &sTanVal2  );  if( error EQ NL_YES ) { NL_OUT; }
  error = N_CrvDerivs( curLR[0], V[0], NL_LEFT, 1, CD1      );  if( error EQ NL_YES ) { NL_OUT; }
  error = N_CrvDerivs( curLR[1], V[0], NL_LEFT, 1, CD2      );  if( error EQ NL_YES ) { NL_OUT; }

  lMisMatch = 0;
  N_VectorDiff( sTanVal1, CD1[1], &A );
  N_VectorDiff( sTanVal2, CD2[1], &B );
  N_VectorMagnitude( A, &da ); if ( da GT MTOL ) { lMisMatch = 1; }
  N_VectorMagnitude( B, &db ); if ( db GT MTOL ) { lMisMatch = 1; }

  if ( lMisMatch > 0 )
  {
      N_VectorMagnitude( sTanVal1, &da );
      N_VectorMagnitude( CD1[1],   &db );
      dScale0 = db / da;
      N_VectorMagnitude( sTanVal2, &da );
      N_VectorMagnitude( CD2[1],   &db );
      dScale1 = db / da;
      N_LinearMultiplyCrv( dScale0, dScale1, curBT[2] );
      lMisMatch = 0;

      // Recheck them.
      error = N_CrvEval  ( curBT[2], U[0], NL_LEFT,  &sTanVal1  );  if( error EQ NL_YES ) { NL_OUT; }
      error = N_CrvEval  ( curBT[2], U[r], NL_LEFT,  &sTanVal2  );  if( error EQ NL_YES ) { NL_OUT; }
      N_VectorDiff( sTanVal1, CD1[1], &A );
      N_VectorDiff( sTanVal2, CD2[1], &B );
      N_VectorMagnitude( A, &da );  if( da GT MTOL ) { lMisMatch = 1; }
      N_VectorMagnitude( B, &db );  if( db GT MTOL ) { lMisMatch = 1; }
      if ( lMisMatch > 0 )
        { lWarning = 1; }
  }



  /* Check curBT[3]: derT. */
  error = N_CrvEval  ( curBT[3], U[0], NL_LEFT,  &sTanVal1  );  if( error EQ NL_YES ) { NL_OUT; }
  error = N_CrvEval  ( curBT[3], U[r], NL_LEFT,  &sTanVal2  );  if( error EQ NL_YES ) { NL_OUT; }
  error = N_CrvDerivs( curLR[0], V[s], NL_LEFT, 1, CD1      );  if( error EQ NL_YES ) { NL_OUT; }
  error = N_CrvDerivs( curLR[1], V[s], NL_LEFT, 1, CD2      );  if( error EQ NL_YES ) { NL_OUT; }

  lMisMatch = 0;
  N_VectorDiff( sTanVal1, CD1[1], &A );
  N_VectorDiff( sTanVal2, CD2[1], &B );
  N_VectorMagnitude( A, &da );  if( da GT MTOL ) { lMisMatch = 1; }
  N_VectorMagnitude( B, &db );  if( db GT MTOL ) { lMisMatch = 1; }

  if ( lMisMatch > 0 )
  {
      N_VectorMagnitude( sTanVal1, &da );
      N_VectorMagnitude( CD1[1],   &db );
      dScale0 = db / da;
      N_VectorMagnitude( sTanVal2, &da );
      N_VectorMagnitude( CD2[1],   &db );
      dScale1 = db / da;
      N_LinearMultiplyCrv( dScale0, dScale1, curBT[3] );
      lMisMatch = 0;

      // Recheck them.
      error = N_CrvEval  ( curBT[3], U[0], NL_LEFT,  &sTanVal1  );  if( error EQ NL_YES ) { NL_OUT; }
      error = N_CrvEval  ( curBT[3], U[r], NL_LEFT,  &sTanVal2  );  if( error EQ NL_YES ) { NL_OUT; }
      N_VectorDiff( sTanVal1, CD1[1], &A );
      N_VectorDiff( sTanVal2, CD2[1], &B );
      N_VectorMagnitude( A, &da );  if( da GT MTOL ) { lMisMatch = 1; }
      N_VectorMagnitude( B, &db );  if( db GT MTOL ) { lMisMatch = 1; }
      if ( lMisMatch > 0 )
        { lWarning = 1; }
      lMisMatch = 0;
  }



  /*
   * Now compute the twist vectors to use.
   * These are the mixed partials (Suv) at the four corners.
   * These would be the first derivatives of the CBD curves at the corners.
   * The values of corresponding derivatives should match: for example,
   * the derivative of the bottom CBD curve at its start (the lower left
   * corner) should match the derivative of the left CBD at its start.
   * If they don't match, the surface might not meet the boundaries smoothly,
   * and we set lWarning to 2 and continue on.
   * In any case, we use the average of the two derivatives.
   */

  error = N_CrvDerivs( curLR[2], V[0], NL_LEFT, 1, CD1);  if( error EQ NL_YES ) { NL_OUT; }
  error = N_CrvDerivs( curBT[2], U[0], NL_LEFT, 1, CD2);  if( error EQ NL_YES ) { NL_OUT; }

  N_VectorDiff( CD1[1], CD2[1], &A );  /* These derivs should be the same. */
  N_VectorMagnitude( A, &mag );
  if ( mag GT dTwistTol )  { lWarning = 2; }

  N_VectorCombine( 0.5, CD1[1], 0.5, CD2[1], &T00 );  /* Average of two vectors */

  error = N_CrvDerivs( curBT[2], U[r], NL_LEFT, 1, CD1 );  if( error EQ NL_YES ) { NL_OUT; }
  error = N_CrvDerivs( curLR[3], V[0], NL_LEFT, 1, CD2 );  if( error EQ NL_YES ) { NL_OUT; }

  N_VectorDiff( CD1[1], CD2[1], &A );
  N_VectorMagnitude( A, &mag );
  if ( mag GT dTwistTol )  { lWarning = 2; }

  N_VectorCombine( 0.5, CD1[1], 0.5, CD2[1], &T10 );

  error = N_CrvDerivs( curLR[3], V[s], NL_LEFT, 1, CD1 );  if( error EQ NL_YES ) { NL_OUT; }
  error = N_CrvDerivs( curBT[3], U[r], NL_LEFT, 1, CD2 );  if( error EQ NL_YES ) { NL_OUT; }

  N_VectorDiff( CD1[1], CD2[1], &A );
  N_VectorMagnitude( A, &mag );
  if ( mag GT dTwistTol )  { lWarning = 2; }

  N_VectorCombine( 0.5, CD1[1], 0.5, CD2[1], &T11 );

  error = N_CrvDerivs( curBT[3], U[0], NL_LEFT, 1, CD1 );  if( error EQ NL_YES ) { NL_OUT; }
  error = N_CrvDerivs( curLR[2], V[s], NL_LEFT, 1, CD2 );  if( error EQ NL_YES ) { NL_OUT; }

  N_VectorDiff( CD1[1], CD2[1], &A );
  N_VectorMagnitude( A, &mag );
  if ( mag GT dTwistTol )  { lWarning = 2; }

  N_VectorCombine( 0.5, CD1[1], 0.5, CD2[1], &T01 );


  /********************************/
  /* Compute the various surfaces */
  /********************************/

  /* Allocate memory */

  N_CrvGetCPtsDegreeAndKnots( curLR[0], &m, &Aw, &q, &s, &VA );
  N_CrvGetCPtsDegreeAndKnots( curBT[0], &n, &Aw, &p, &r, &UA );

  error = N_AllocSrfArrays( &surU, 3, m, 3, q, 7, s, &SL );  if( error EQ NL_YES ) { NL_OUT; }
  error = N_AllocSrfArrays( &surV, n, 3, p, 3, r, 7, &SL );  if( error EQ NL_YES ) { NL_OUT; }
  error = N_AllocSrfArrays( &surT, 3, 3, 3, 3, 7, 7, &SL );  if( error EQ NL_YES ) { NL_OUT; }

  /* Compute u-blend */

  N_SrfGetCPtsAndKnots( &surU, &Uw, &U, &V );

  N_CrvGetCPts( curLR[0], &m, &Aw );
  for ( j=0; j<=m; j++ ) { N_CopyCPt( Aw[j], &Uw[0][j] ); }

  N_CrvGetCPts( curLR[1], &m, &Aw );
  for ( j=0; j<=m; j++ ) { N_CopyCPt( Aw[j], &Uw[3][j] ); }

  N_CrvGetCPts( curLR[2], &m, &Aw );
  for ( j=0; j<=m; j++ ) { N_Combine2CPts( 1.0, Uw[0][j],  one_3, Aw[j], &Uw[1][j] ); }

  N_CrvGetCPts( curLR[3], &m, &Aw );
  for ( j=0; j<=m; j++ ) { N_Combine2CPts( 1.0, Uw[3][j], -one_3, Aw[j], &Uw[2][j] ); }

  for ( i=0; i<=3; i++ ) {  U[i] = 0.0;  U[4+i] = 1.0;  }
  for ( j=0; j<=s; j++ ) {  V[j] = VA[j]; }

  /* Compute v-blend */

  N_SrfGetCPtsAndKnots( &surV, &Vw, &U, &V );

  N_CrvGetCPts( curBT[0], &n, &Aw );
  for ( i=0; i<=n; i++ )  N_CopyCPt( Aw[i], &Vw[i][0]);

  N_CrvGetCPts( curBT[1], &n, &Aw );
  for ( i=0; i<=n; i++ )  N_CopyCPt( Aw[i], &Vw[i][3]);

  N_CrvGetCPts( curBT[2], &n, &Aw );
  for ( i=0; i<=n; i++ )  N_Combine2CPts( 1.0, Vw[i][0],  one_3, Aw[i], &Vw[i][1] );

  N_CrvGetCPts( curBT[3], &n, &Aw );
  for ( i=0; i<=n; i++ )  N_Combine2CPts( 1.0, Vw[i][3], -one_3, Aw[i], &Vw[i][2] );

  for ( i=0; i<=r; i++ )  {  U[i] = UA[i]; }
  for ( j=0; j<=3; j++ )  {  V[j] = 0.0;  V[4+j] = 1.0;  }

  /* Compute tensor product surface */

  N_SrfGetCPtsAndKnots( &surT, &Tw, &U, &V );

  for( i=0; i<=3; i++ )
  {
    N_CopyCPt( Uw[i][0], &Tw[i][0] );
    N_CopyCPt( Uw[i][m], &Tw[i][3] );
  }

  for( j=1; j<=2; j++ )
  {
    N_CopyCPt( Vw[0][j], &Tw[0][j] );
    N_CopyCPt( Vw[n][j], &Tw[3][j] );
  }

  N_PtToCPt( T00, &T00w );
  N_PtToCPt( T10, &T10w );
  N_PtToCPt( T01, &T01w );
  N_PtToCPt( T11, &T11w );

  N_Combine4CPts(  one_9, T00w, 1.0, Tw[1][0], 1.0, Tw[0][1], -1.0, Tw[0][0], &Tw[1][1] );
  N_Combine4CPts( -one_9, T10w, 1.0, Tw[3][1], 1.0, Tw[2][0], -1.0, Tw[3][0], &Tw[2][1] );
  N_Combine4CPts( -one_9, T01w, 1.0, Tw[1][3], 1.0, Tw[0][2], -1.0, Tw[0][3], &Tw[1][2] );
  N_Combine4CPts(  one_9, T11w, 1.0, Tw[2][3], 1.0, Tw[3][2], -1.0, Tw[3][3], &Tw[2][2] );

  for( i=0; i<=3; i++ )  {  U[i] = 0.0;  U[4+i] = 1.0;  }
  for( j=0; j<=3; j++ )  {  V[j] = 0.0;  V[4+j] = 1.0;  }


  /* Make surfaces compatible */

  surA = N_AllocArraySrfPtrs( 2, &SL );  if ( surA EQ NULL ) { NL_QUIT; }

  surA[0] = &surU;
  surA[1] = &surV;
  surA[2] = &surT;

  error = N_MakeSrfsCompatibleUV( surA, 2, &SL );  if( error EQ NL_YES ) { NL_OUT; }


  /* Compute output surface */

  N_SrfGetCPtsDegreesAndKnots( surA[0], &n, &m, &Uw, &p, &q, &r, &s, &UA, &VA );

  error = N_SrfSizeArrays( sur, n, m, p, q, r, s, rname, SS );
  if( error EQ NL_YES )  NL_OUT;

  N_SrfGetCPts( surA[1], &n ,&m, &Vw);
  N_SrfGetCPts( surA[2], &n ,&m, &Tw);
  N_SrfGetCPtsAndKnots( sur, &Pw, &U, &V );

  for ( i=0; i<=n; i++ )
  {
    for ( j=0; j<=m; j++ )
    {
      N_TranslateSum2CPts( Uw[i][j], 1.0, Vw[i][j], -1.0, Tw[i][j], &Pw[i][j] );
    }
  }

  for ( i=0; i<=r; i++ ) { U[i] = UA[i]; }
  for ( j=0; j<=s; j++ ) { V[j] = VA[j]; }


  /* End NURBS and Exit */


  EXIT:

  N_EndNurbs(&SL);

  return(error);

} // end sm_CreateCoonsSrf

/*******************************************************************//**
PURPOSE: Create a coons surface from 4 boundary curves & derivative
    curves if available.

USAGE NOTES --- 
   Rational curves are not allowed: all curves must be nonrational.

NOTES: The boundary curves are assumed to intersect
   at their respective end points.  The boundary curves and the corresponding
   cross-boundary derivative curves ("CBD's"), if supplied, are assumed to be
   compatible in the B-Spline sense.  Also, the directions of the CBD's must
   match the derivatives of the corresponding boundary curves at their ends:
   for example, the value of the bottom CBD curve at its beginning must be
   parallel to the first derivative of the left boundary curve at its beginning.
   If these conditions are not met, then the resulting surface might not match
   the CBD's, i.e., it might not meet adjacent surfaces smoothly.  Note
   however that the magnitudes of those derivatives need not match: this
   routine will scale the CBD curves in place to match the derivatives of
   the boundary curves where they meet.
    
   In the 3-sided case, one of the curves should be a 'NL_POINT '
   curve and its derivative curve is suggested to be a linear interpolation
   between two end tangent "values" from two adjacent curves.

   If we are creating bilinear coons patch, the derivative
   curves need not be given.

   This method is only available for users with NLib
   The list of boundary curves need to be ordered:

      Left, Right, Bottom, Top

   The left, right curves should be going in the 'same'
   direction (you may need to reverse the 'right' curve).
   The bottom, top curves should be going in the 'same'
   direction (you may need to reverse the 'top' curve).

   The left and right curves must have the same domain   
        Left (t0:t1)  =  Right(t0:t1)
   same for top, bottom.

   ( If you draw a rectangle, put parallel arrows to the
   right on the top and bottom, and pointing up on the sides)

   The curves do need to intersect at the  corners.

  
***********************************************************************/
SmStatus SmBSplineSurface::CreateCoonsPatch
  (const SmContext                 & crContext,             // in : context for new object construction
   double                            dTol,                  // in : max allowed patch/boundary curve deviation
   SmBoolean                         bCreateBilinearPatch,  // in : TRUE = Create bilinear patch (ignore crDerivCrvs)
                                                            //      FALSE= Create bicubic patch through curves and crossTangents
   const SmTArray<SmBSplineCurve*> & crBdryCrvs,            // in : ordered array of 4 boundary curves
   const SmTArray<SmBSplineCurve*> & crDerivCrvs,           // in : associated array of 4 crossTangent functions
   SmBSplineSurface               *& rpCoonsSurface)        // out: the new coons patch 
{
  // check input - 4 curves 
  SM_ASSERT(crBdryCrvs.GetSize() == 4);
  ULONG lNumDerivCrvs = crDerivCrvs.GetSize();
  SM_ASSERT(lNumDerivCrvs == 0 || lNumDerivCrvs == 4);

  // locals
  NL_CURVE    *curL = NULL, *derL = NULL, *curR = NULL, *derR = NULL, *curB = NULL, *derB = NULL;
  NL_CURVE    *curT = NULL, *derT = NULL;
  NL_SURFACE  sur;
  NL_STACKS   SC, SS;
  SmNLibStackHandler sSH(&SC);
  SmNLibStackHandler sSH1(&SS);

  // Get boundary curves and derivatives
  curL = crBdryCrvs[0]->GetOrCreateGwNurbPointer();
  curR = crBdryCrvs[1]->GetOrCreateGwNurbPointer();
  curB = crBdryCrvs[2]->GetOrCreateGwNurbPointer();
  curT = crBdryCrvs[3]->GetOrCreateGwNurbPointer();

  // when given crossTangent functions
  if (lNumDerivCrvs == 4) 
    {
      derL = (NL_CURVE*)crDerivCrvs[0]->GetOrCreateGwNurbPointer();
      derR = (NL_CURVE*)crDerivCrvs[1]->GetOrCreateGwNurbPointer();
      derB = (NL_CURVE*)crDerivCrvs[2]->GetOrCreateGwNurbPointer();
      derT = (NL_CURVE*)crDerivCrvs[3]->GetOrCreateGwNurbPointer();
    }
  else if (bCreateBilinearPatch) 
    {
      /* // NOTE THESE ARE NOT NEEDED
      // Make cross-boundary derivative curve
      // for bilinear coons patch
      VECTOR derS[4], derE[4], norm[4];
      for (ULONG ii=0; ii<4; ii++) {
          SmBSplineCurve * pCurve = crBdryCrvs[ii];
          SmExtent1d sIvl = pCurve->GetNaturalInterval();
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
          if (bDebugMe) {
              if (ii==0) smgfx_Erase();
              smgfx_ChangeColor();
              pCurve->DrawAt(sIvl.GetMin(),0);
              pCurve->Draw();
              sm_GraphicsLoop();
          }
#endif

          SmVector3d sPV[2];
          SER(pCurve->Evaluate(sIvl.GetMin()+SM_EFF_ZERO_SQRT,1,TRUE,sPV));
          COPY_XYZ(sPV[1],derS[ii]);

          SER(pCurve->Evaluate(sIvl.GetMax()-SM_EFF_ZERO_SQRT,1,TRUE,sPV));
          COPY_XYZ(sPV[1],derE[ii]);
      }
      N_VectorCross(derS[0],derS[2],&norm[0]); //normal at LB corner
      NL_SER(N_VectorNormalizeRef(&norm[0]));
      N_VectorCross(derE[0],derS[3],&norm[1]); //normal at LT corner
      NL_SER(N_VectorNormalizeRef(&norm[1]));
      N_VectorCross(derE[1],derE[3],&norm[2]); //normal at RT corner
      NL_SER(N_VectorNormalizeRef(&norm[2]));
      N_VectorCross(derS[1],derE[2],&norm[3]); //normal at RB corner
      NL_SER(N_VectorNormalizeRef(&norm[3]));
      NL_CURVE   derCurL, derCurR, derCurB, derCurT;
      NL_SER(N_CreateCrossBoundaryDerivCrv(curL,norm[0],norm[1],derS[2],derS[3],&derCurL,&SC));
      NL_SER(N_CreateCrossBoundaryDerivCrv(curR,norm[3],norm[2],derE[2],derE[3],&derCurR,&SC));
      NL_SER(N_CreateCrossBoundaryDerivCrv(curB,norm[0],norm[3],derS[0],derS[1],&derCurB,&SC));
      NL_SER(N_CreateCrossBoundaryDerivCrv(curT,norm[1],norm[2],derE[0],derE[1],&derCurT,&SC));
      derL = &derCurL;
      derR = &derCurR;
      derB = &derCurB;
      derT = &derCurT;
      */
    }
  else 
    {
      SER(SM_ERR); // Missing derivative curves
    }

#ifdef SM_DEBUG_CODE
ULONG di ;
SmBoolean bDebugMe = FALSE;
  if (bDebugMe) 
    {
      smgfx_SetColor(1,0,0);
      for ( di=0; di<4; di++) 
        {
          smgfx_ChangeColor(di != 0);
          crBdryCrvs[di]->DrawWDeriv(crBdryCrvs[di]->GetNaturalInterval(),0); sm_GraphicsLoop();
          crBdryCrvs[di]->Dump();
          if (lNumDerivCrvs == 4) 
            {
              crDerivCrvs[di]->DrawWDeriv(crDerivCrvs[di]->GetNaturalInterval(),0); sm_GraphicsLoop();
              crDerivCrvs[di]->Dump();
            }
          sm_GraphicsLoop();
        }
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // create patch branching on bCreateBilinearPatch flag
  N_SrfInitArrays(&sur);
  ULONG lWarning;
  if (bCreateBilinearPatch) 
    { // DOES NOT USE DERIVS
      NL_CURVE  *nlCurU[2] = { curL, curR };
      NL_CURVE  *nlCurV[2] = { curB, curT };
      NL_SER(N_CreateCoonsSrf(nlCurU,nlCurV,&sur,&SC,&SS));
    }
  else 
    {
      NL_SER(sm_CreateCoonsSrf(dTol,curL,derL,curR,derR,curB,derB,curT,derT,
                         &sur,&SC,&SS,lWarning));
      if (lWarning > 0) 
        { 
          SM_DBG_WARN(_T("Possible Problem In Coons Patch Creation - Ignoring\n"));
        }
    }

  // copy the new surface into an SMLib object to get contiguous memory management
  rpCoonsSurface = new (crContext) SmBSplineSurface((gw_SURFACE *)&sur) ;

#ifdef SM_DEBUG_CODE
  if (bDebugMe) 
    {
      smgfx_Erase();
      rpCoonsSurface->DrawUV(4,4);
      sm_GraphicsLoop();
    }
#endif

  // all done
  return SM_SUCCESS;

} // end SmBSplineSurface::CreateCoonsPatch

/*******************************************************************//**
PURPOSE: Create an extended surface by extending a given distance from
    each side of the original surface

NOTES: This method is only available for users with NLib
    
***********************************************************************/
SmStatus SmBSplineSurface::CreateExtendedSurface
  (const SmContext   & crContext,            // in : context for new object construction
   double              dDist,                // in : Distance of extension from each side
   SmContinuityType    eExtensionContinuity, // in : OneOf: SM_CT_G1 - linear extension
                                             //             SM_CT_G1R - 
                                             //             SM_CT_G1_G2 - extension with second derivative
                                             //             SM_CT_CINFINITY - infinite continuity
   SmSurface *& rpExtended)                  // out: the newly constructed surface (NULL on input)
{
  // pass the call along
  return (CreateExtendedSurface( crContext, SM_SP_BOTH, dDist, eExtensionContinuity, rpExtended));

} // end SmBSplineSurface::CreateExtendedSurface

/*******************************************************************//**
PURPOSE: Create an extended surface by extending a given distance from
    one or all side(s) of the original surface. This is an overloaded function.

NOTES: This method is only available for users with NLib
    
***********************************************************************/
SmStatus SmBSplineSurface::CreateExtendedSurface
 (const SmContext   & crContext,            // in : context for new object construction
  SmSurfParamType     eExtDirection,        // in : Either extend in SM_SP_U or SM_SP_V direction
                                            //      or SM_SP_UMIN/VMIN/UMAX/VMAX/BOTH
  double              dDist,                // in : Distance of extension from each side
  SmContinuityType    eExtensionContinuity, // in : oneof: SM_CT_G1 - linear extension
                                            //             SM_CT_G1R - 
                                            //             SM_CT_G1_G2 - extension with second derivative
                                            //             SM_CT_CINFINITY - infinite continuity
  SmSurface *& rpExtended)                  // out: the newly constructed surface (NULL on input)
{
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  if (bDebugMe) 
    {
      SM_DUMP_AND_ASSERT_VALID(this) ;

      SmFace * pFace = (SmFace *)GetFace() ;
      SmBrep * pBrep = pFace ? pFace->GetBrep() : NULL ;

      smgfx_Erase();
      smgfx_SetLook(1,2, 0,0,1); if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,1); DrawUV(1,1); sm_GraphicsLoop();
      smgfx_SetLook(1,2, 0,0,0) ; if(pFace) pFace->DrawUV(8,8,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ; 
      if (FALSE) 
        { smgfx_Erase();
          DrawNet(); sm_GraphicsLoop();
        }
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // locals
  NL_STACKS nlStacks;
  SmNLibStackHandler sSH(&nlStacks);

  // copy input surface into local objecct
  NL_SURFACE * sur       = GetOrCreateGwNurbPointer();
  NL_SURFACE   esur;
  N_SrfInitArrays(&esur);
  N_SrfCopy(sur,&esur,&nlStacks);

  // locals
  SmExtent2d      sUVDomain = GetNaturalUVDomain();
  double          dExtDist = dDist;

  // pick continuity level
  NL_FLAG gflg =   eExtensionContinuity == SM_CT_G1     ? NL_G1
              : eExtensionContinuity == SM_CT_G1R       ? NL_G1R 
              : eExtensionContinuity == SM_CT_G1_G2     ? NL_G2 
              : eExtensionContinuity == SM_CT_CINFINITY ? NL_CMAX
              :                                           NL_G1;

  // sort out where extension is needed
  SmBoolean       bUDir     = (   eExtDirection == SM_SP_U    
                               || eExtDirection == SM_SP_UMIN 
                               || eExtDirection == SM_SP_UMAX) ;
  SmBoolean       bVDir     = (   eExtDirection == SM_SP_V    
                               || eExtDirection == SM_SP_VMIN 
                               || eExtDirection == SM_SP_VMAX) ;
  SmSurfParamType eExtUorV  =   bUDir ? SM_SP_U
                              : bVDir ? SM_SP_V
                              :         SM_SP_BOTH ;
  SmBoolean       bSkipUMIN = (   eExtDirection == SM_SP_NEITHER
                               || bVDir
                               || eExtDirection == SM_SP_UMAX) ;
  SmBoolean       bSkipVMIN = (   eExtDirection == SM_SP_NEITHER
                               || bUDir
                               || eExtDirection == SM_SP_VMAX) ;
  SmBoolean       bSkipUMAX = (   eExtDirection == SM_SP_NEITHER
                               || bVDir
                               || eExtDirection == SM_SP_UMIN) ;
  SmBoolean       bSkipVMAX = (   eExtDirection == SM_SP_NEITHER
                               || bUDir
                               || eExtDirection == SM_SP_VMIN) ;

  // Don't extend through singularities in U or V
  SmSurfParamType eSingDirection = SM_SP_UNKNOWN ;
  if(IsSingularity(sUVDomain.GetMin(), eSingDirection)) 
    {
      if (eSingDirection == SM_SP_U) { bSkipVMIN = TRUE; }
      else                           { bSkipUMIN = TRUE; }
    }

  if (IsSingularity(sUVDomain.GetMax(), eSingDirection)) 
    {
      if (eSingDirection == SM_SP_U) { bSkipVMAX = TRUE; }
      else                           { bSkipUMAX = TRUE; }
    }

  // when asked and allowed - extend in U direction
  if(   (   (eExtUorV == SM_SP_U ) 
         || (eExtUorV == SM_SP_BOTH))
     && !IsClosed(sUVDomain,SM_SP_U)) 
    {
      // when extending in the UMin direction
      if (!bSkipUMIN) 
        {
          // extend Surf in UMin - shorter dists until extension succeeds
          while(0 != N_SrfExtendByDist(&esur, dExtDist, // rtn: 0 = No Error, else Error
                                       NL_UDIR,  NL_START,
                                       gflg,  &esur,
                                       &nlStacks,   &nlStacks))  
            {
              dExtDist = dExtDist/2.0;
              if (dExtDist < dDist/100.0) 
                { SER(SM_ERR); }
            }
          
          // quit - when extended surface is no good  
          if(0 != N_SrfIsValid(&esur,rname)) // rtn: 0 = No Error, else Error
            { SER(SM_ERR); }
          
          // if needed - multiple extensions until Extension gets to be near requested size  
          while (dExtDist < dDist/2.0) 
            {
              dExtDist *= 2.0;
              NL_SER(0 != N_SrfExtendByDist(&esur, dExtDist, // rtn: 0 = No Error, else Error
                                            NL_UDIR,  NL_START,
                                            gflg,  &esur,
                                            &nlStacks, &nlStacks));
              if(0 != N_SrfIsValid(&esur,rname)) // rtn: 0 = No Error, else Error
                { SER(SM_ERR); }
            } // end need multiple extensions when 1st extension dist was shortened
            
#ifdef SM_DEBUG_CODE
          if(bDebugMe)
            {
              SmBSplineSurface * pTemp = new (crContext) SmBSplineSurface((gw_SURFACE *)&esur) ; // gwc: was TRUE = run AssertValidAndHeal on New BSplineSurface
              SmObjDelete sClean(pTemp);
              SM_DUMP_AND_ASSERT_VALID(pTemp) ;

              SmFace * pFace = (SmFace *)GetFace() ;
              SmBrep * pBrep = pFace ? pFace->GetBrep() : NULL ;

              smgfx_Erase();
              smgfx_SetLook(1,2, 0,0,1); if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
              smgfx_SetLook(1,2, 0,0,1); DrawUV(1,1); sm_GraphicsLoop();
              smgfx_SetLook(1,2, 0,0,0) ; if(pFace) pFace->DrawUV(8,8,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ; 
              smgfx_SetLook(1,2, 0,1,1); pTemp->DrawUV(1,1); sm_GraphicsLoop();
              smgfx_SetLook(1,2, 0,1,1); pTemp->DrawVectorField(pTemp->GetNaturalUVDomain(),25,25,SM_DM_U_SCALED); sm_GraphicsLoop();
              smgfx_SetLook(1,2, 0,1,1); pTemp->DrawVectorField(pTemp->GetNaturalUVDomain(),25,25,SM_DM_V_SCALED); sm_GraphicsLoop();
              smgfx_SetLook(1,2, 0,1,0); pTemp->DrawPolygon(); sm_GraphicsLoop();
              smgfx_SetLook(4,5, 1,0,0); pTemp->DrawParams(); sm_GraphicsLoop();
              if (FALSE) 
                { smgfx_Erase();
                  pTemp->DrawNet(); sm_GraphicsLoop();
                }
              sm_GraphicsLoop();
              smgfx_SetColor(0,0,0);
            }
#endif // SM_DEBUG_CODE
        } // end extending in the UMin direction
      
      // reset the extension distance  
      dExtDist = dDist;
      
      // when asked - extend in the UMax direction  
      if (!bSkipUMAX) 
        {
          // extend Surf in UMax - shorter dists until extension succeeds
          while(0 != N_SrfExtendByDist(&esur, dExtDist, // rtn: 0 = No Error, else Error
                                       NL_UDIR,  NL_END,
                                       gflg,  &esur,
                                       &nlStacks,   &nlStacks)) 
            {
              dExtDist = dExtDist/2.0;
              if (dExtDist < dDist/100.0) 
                { SER(SM_ERR); }
            }
            
          // quit - when extended surface is no good  
          if(0 != N_SrfIsValid(&esur,rname)) // rtn: 0 = No Error, else Error
            { SER(SM_ERR); }
            
          // if needed - multiple extensions until Extension gets to be near requested size  
          while (dExtDist < dDist/2.0) 
            {
              dExtDist *= 2.0;
              NL_SER(0 != N_SrfExtendByDist(&esur, dExtDist, // rtn: 0 = No Error, else Error
                                            NL_UDIR,  NL_END,
                                            gflg,  &esur,
                                            &nlStacks,   &nlStacks));
              if(0 != N_SrfIsValid(&esur,rname)) // rtn: 0 = No Error, else Error
                { SER(SM_ERR); }
            }
#ifdef SM_DEBUG_CODE
          if(bDebugMe)
            {
              SmBSplineSurface * pTemp = new (crContext) SmBSplineSurface((gw_SURFACE *)&esur) ; // gwc: was TRUE = run AssertValidAndHeal on New BSplineSurface
              SmObjDelete sClean(pTemp);
              SM_DUMP_AND_ASSERT_VALID(pTemp) ;

              SmFace * pFace = (SmFace *)GetFace() ;
              SmBrep * pBrep = pFace ? pFace->GetBrep() : NULL ;

              smgfx_Erase();
              smgfx_SetLook(1,2, 0,0,1); if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
              smgfx_SetLook(1,2, 0,0,1); DrawUV(1,1); sm_GraphicsLoop();
              smgfx_SetLook(1,2, 0,0,0) ; if(pFace) pFace->DrawUV(8,8,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ; 
              smgfx_SetLook(1,2, 0,1,1); pTemp->DrawUV(1,1); sm_GraphicsLoop();
              smgfx_SetLook(1,2, 0,1,1); pTemp->DrawVectorField(pTemp->GetNaturalUVDomain(),25,25,SM_DM_U_SCALED); sm_GraphicsLoop();
              smgfx_SetLook(1,2, 0,1,1); pTemp->DrawVectorField(pTemp->GetNaturalUVDomain(),25,25,SM_DM_V_SCALED); sm_GraphicsLoop();
              smgfx_SetLook(1,2, 0,1,0); pTemp->DrawPolygon(); sm_GraphicsLoop();
              smgfx_SetLook(4,5, 1,0,0); pTemp->DrawParams(); sm_GraphicsLoop();
              if (FALSE) 
                { smgfx_Erase();
                  pTemp->DrawNet(); sm_GraphicsLoop();
                }
              sm_GraphicsLoop();
              smgfx_SetColor(0,0,0);
            }
#endif // SM_DEBUG_CODE
        } // end extend in the UMax direction check
    } // end extend in U direction check
  
  // when asked - extend in the V direction  
  if(   (   (eExtUorV == SM_SP_V) 
         || (eExtUorV == SM_SP_BOTH)) 
     &&  !IsClosed(sUVDomain,SM_SP_V)) 
    {
      // reset extension distance  
      dExtDist = dDist;
      
      // when asked - extend in the V Min direction  
      if (!bSkipVMIN) 
        {
          // extend Surf in VMin - shorter dists until extension succeeds
          while(0 != N_SrfExtendByDist(&esur, dExtDist, // rtn: 0 = No Error, else Error
                                       NL_VDIR,  NL_START,
                                       gflg,  &esur,
                                       &nlStacks,   &nlStacks)) 
            {
              dExtDist = dExtDist/2.0;
              if (dExtDist < dDist/100.0) 
                { SER(SM_ERR); }
            }

          // quit - when extended surface is no good  
          if(0 != N_SrfIsValid(&esur,rname)) // rtn: 0 = No Error, else Error
            { SER(SM_ERR); }
            
          // if needed - multiple extensions until Extension gets to be near requested size  
          while (dExtDist < dDist/2.0) 
            {
              dExtDist *= 2.0;
              NL_SER(0 != N_SrfExtendByDist(&esur, dExtDist, // rtn: 0 = No Error, else Error
                                            NL_VDIR,  NL_START,
                                            gflg,  &esur,
                                            &nlStacks,   &nlStacks));
              if(0 != N_SrfIsValid(&esur,rname)) // rtn: 0 = No Error, else Error 
                { SER(SM_ERR); }
            } // end need multiple extensions when 1st extension dist was shortened

#ifdef SM_DEBUG_CODE
          if(bDebugMe)
            {
              SmBSplineSurface * pTemp = new (crContext) SmBSplineSurface((gw_SURFACE *)&esur) ; // gwc: was TRUE = run AssertValidAndHeal on New BSplineSurface
              SmObjDelete sClean(pTemp);
              SM_DUMP_AND_ASSERT_VALID(pTemp) ;

              SmFace * pFace = (SmFace *)GetFace() ;
              SmBrep * pBrep = pFace ? pFace->GetBrep() : NULL ;

              smgfx_Erase();
              smgfx_SetLook(1,2, 0,0,1); if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
              smgfx_SetLook(1,2, 0,0,1); DrawUV(1,1); sm_GraphicsLoop();
              smgfx_SetLook(1,2, 0,0,0) ; if(pFace) pFace->DrawUV(8,8,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ; 
              smgfx_SetLook(1,2, 0,1,1); pTemp->DrawUV(1,1); sm_GraphicsLoop();
              smgfx_SetLook(1,2, 0,1,1); pTemp->DrawVectorField(pTemp->GetNaturalUVDomain(),25,25,SM_DM_U_SCALED); sm_GraphicsLoop();
              smgfx_SetLook(1,2, 0,1,1); pTemp->DrawVectorField(pTemp->GetNaturalUVDomain(),25,25,SM_DM_V_SCALED); sm_GraphicsLoop();
              smgfx_SetLook(1,2, 0,1,0); pTemp->DrawPolygon(); sm_GraphicsLoop();
              smgfx_SetLook(4,5, 1,0,0); pTemp->DrawParams(); sm_GraphicsLoop();
              if (FALSE) 
                { smgfx_Erase();
                  pTemp->DrawNet(); sm_GraphicsLoop();
                }
              sm_GraphicsLoop();
              smgfx_SetColor(0,0,0);
            }
#endif // SM_DEBUG_CODE
        } // end extending in the VMin direction

      // reset the extension distance  
      dExtDist = dDist;
        
      // when asked - extend in the V Max direction
      if (!bSkipVMAX) 
        {
          // extend Surf in VMax - shorter dists until extension succeeds
          while(0 != N_SrfExtendByDist(&esur, dExtDist, // rtn: 0 = No Error, else Error
                                       NL_VDIR,  NL_END,
                                       gflg,  &esur,
                                       &nlStacks,   &nlStacks)) 
            { dExtDist = dExtDist/2.0;
              if(dExtDist < dDist/100.0) 
                { SER(SM_ERR); }
            }
            
          // quit - when extended surface is no good  
          if(0 != N_SrfIsValid(&esur,rname)) // rtn: 0 = No Error, else Error 
            { SER(SM_ERR); }
            
          // if needed - multiple extensions until Extension gets to be near requested size  
          while (dExtDist < dDist/2.0) 
            {
              dExtDist *= 2.0;
              NL_SER(N_SrfExtendByDist(&esur, dExtDist, // rtn: 0 = No Error, else Error
                                       NL_VDIR,  NL_END,
                                       gflg,  &esur,
                                       &nlStacks,   &nlStacks));
              if(0 != N_SrfIsValid(&esur,rname)) // rtn: 0 = No Error, else Error 
                { SER(SM_ERR); }
            }
#ifdef SM_DEBUG_CODE
          if(bDebugMe)
            {
              SmBSplineSurface * pTemp = new (crContext) SmBSplineSurface((gw_SURFACE *)&esur) ; // gwc: was TRUE = run AssertValidAndHeal on New BSplineSurface
              SmObjDelete sClean(pTemp);
              SM_DUMP_AND_ASSERT_VALID(pTemp) ;

              SmFace * pFace = (SmFace *)GetFace() ;
              SmBrep * pBrep = pFace ? pFace->GetBrep() : NULL ;

              smgfx_Erase();
              smgfx_SetLook(1,2, 0,0,1); if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
              smgfx_SetLook(1,2, 0,0,1); DrawUV(1,1); sm_GraphicsLoop();
              smgfx_SetLook(1,2, 0,0,0) ; if(pFace) pFace->DrawUV(8,8,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ; 
              smgfx_SetLook(1,2, 0,1,1); pTemp->DrawUV(1,1); sm_GraphicsLoop();
              smgfx_SetLook(1,2, 0,1,1); pTemp->DrawVectorField(pTemp->GetNaturalUVDomain(),25,25,SM_DM_U_SCALED); sm_GraphicsLoop();
              smgfx_SetLook(1,2, 0,1,1); pTemp->DrawVectorField(pTemp->GetNaturalUVDomain(),25,25,SM_DM_V_SCALED); sm_GraphicsLoop();
              smgfx_SetLook(1,2, 0,1,0); pTemp->DrawPolygon(); sm_GraphicsLoop();
              smgfx_SetLook(4,5, 1,0,0); pTemp->DrawParams(); sm_GraphicsLoop();
              if (FALSE) 
                { smgfx_Erase();
                  pTemp->DrawNet(); sm_GraphicsLoop();
                }
              sm_GraphicsLoop();
              smgfx_SetColor(0,0,0);
            }
#endif // SM_DEBUG_CODE

        } // end extending in the V Max direction check
    } // end extend in the V direction check
   
   
  // build output SmBSplineSurface from esur SURFACE object
  if (IsClosed(sUVDomain,SM_SP_U) && IsClosed(sUVDomain,SM_SP_V)) 
       { // surfaces closed in both directions - don't get extended - copy input surface
         rpExtended = new (crContext) SmBSplineSurface(*this);  
       }
  else { // extended surfaces - build output from esur object
         rpExtended = new (crContext) SmBSplineSurface((gw_SURFACE *)&esur) ;
       }

#ifdef SM_DEBUG_CODE
SmBoolean bDebugOutput = FALSE;
  if (bDebugMe) 
    {
      rpExtended->Dump();
      if (bDebugOutput)
        { rpExtended->WriteToFile(_T("surf.smc")); }

      SmFace * pFace = (SmFace *)GetFace() ;
      SmBrep * pBrep = pFace ? pFace->GetBrep() : NULL ;

      smgfx_Erase();
      smgfx_SetLook(1,2, 0,0,1); if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,1); DrawUV(1,1); sm_GraphicsLoop();
      smgfx_SetLook(1,2, 0,0,0) ; if(pFace) pFace->DrawUV(8,8,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ; 
      smgfx_SetLook(1,2, 0,1,1); rpExtended->DrawUV(1,1); sm_GraphicsLoop();
      smgfx_SetLook(1,2, 0,1,1); rpExtended->DrawVectorField(rpExtended->GetNaturalUVDomain(),50,50,SM_DM_V_SCALED); sm_GraphicsLoop();
      smgfx_SetLook(1,2, 0,1,0); rpExtended->DrawPolygon(); sm_GraphicsLoop();
      smgfx_SetLook(4,5, 1,0,0); rpExtended->DrawParams(); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // all done
  return SM_SUCCESS;

} // end SmBSplineSurface::CreateExtendedSurface

/*******************************************************************//**
PURPOSE: Join two surfaces along a common edge

NOTES: This virtual BSplineSurface routine joins two SmBSplineSurfaces at a common boundary.  
   The surfaces will be made compatible in the direction of the 
   boundary (same knots, degree, rationality etc).  Returns SM_ERR if the
   surfaces are not compatible.

   This method has a substantial restriction: the two surfaces must be
   parameterized such that the common boundary curve is the 'top' of the
   domain of the first surface, and the 'bottom' of the second.
   Also, it has to be the same parameter direction (U/V) for both.
   Fortunately, both of these are easily handled with Reverse() and/or SwapUV().

   Param range of the joined surface comes from the input surfaces as:
     This         :[ThisParamMin,  ThisParamMax]  
     SurfaceToJoin:[OtherParamMin, OtherParamMax]
     JoinedSurface:[ThisParamMin,  ThisParamMax+OtherParamMax-OtherParamMin]

  which looks like:
     Surfs to Join:        +---------------+              +--------------------------------+
                      ThisParamMin    ThisParamMax  OtherParamMin                    OtherParamMax
     
     Joined Surface:       +---------------+--------------------------------+
                      ThisParamMin    ThisParamMax      (ThisParamMax+(OtherParamMax-OtherParamMin))
     
   Memory management:
     thisSurface and pSurfaceToJoin are not modified - caller must manage that memory
     rpJoinedSurfaces is a newly allocated surface   - caller must manage that memory.
***********************************************************************/
SmStatus SmBSplineSurface::JoinSurface
(const SmContext     & crContext,         // in: context for new object construction
   SmBSplineSurface  * pSurfaceToJoin,    // in: Surf to join to 'this' surf (neither surf modified)
   SmSurfParamType     eDirFlag,          // in: Direction of parameter flow across join.
                                          //     SM_SP_U = join this->MaxV to pSurfaceToJoin->MinV isocurve (surfs made compatible in v param)
                                          //     SM_SP_V = join this->MaxU to pSurfaceToJoin->MinU isocurve (surfs made compatible in u param)
   double              dTolerance,        // in: Knot removal tolerance.  The knot corresponding
                                          //     to the merged boundary has multiplicity equal
                                          //     to the degree.  Knot removal will be attempted
                                          //     using this tolerance.
   SmBSplineSurface *& rpJoinedSurfaces)  // out: joined surface (newly allocated), NULL on input
                                          //      Joined dir ParamIvl:[ThisParamMin,  ThisParamMax+(OtherParamMax-OtherParamMin)]
{
  NL_SURFACE **surList;
  NL_SURFACE * thisSur  =                 GetOrCreateGwNurbPointer();
  NL_SURFACE * Sur2join = pSurfaceToJoin->GetOrCreateGwNurbPointer();
  NL_SURFACE Joined;
  NL_FLAG    error;
  NL_FLAG    UorV;
  NL_STACKS  nlStacks;
  SmNLibStackHandler sSH(&nlStacks);

  N_SrfInitArrays( &Joined );
  UorV = (NL_FLAG)(2 - eDirFlag);

  // Make surfaces compatible.
  surList = N_AllocArraySrfPtrs(1, &nlStacks);
  surList[0] = thisSur;
  surList[1] = Sur2join;
  error = N_MakeSrfsCompatible(surList, 1, UorV, &nlStacks);
  if (error EQ NL_YES)
    { return (SM_ERR); } // surfaces not compatible, so not joined

  UorV = (NL_FLAG)(eDirFlag + 1);
  error = N_SrfJoin(thisSur, Sur2join, UorV, dTolerance, &Joined, &nlStacks);
  if (error EQ NL_YES)
    { return (SM_ERR); } // join operation failed

  rpJoinedSurfaces = new(crContext) SmBSplineSurface((gw_SURFACE *)&Joined) ;

  return (SM_SUCCESS);

} // end SmBSplineSurface::JoinSurface

/*******************************************************************//**
PURPOSE: Create an N-sided patch from N boundary-curves & N derivative-curves.

NOTES: Some of the boundary curves may not have derivative curves.
    If that is the case a NULL pointer will be in the corresponding 
    place in the crDerivCrvs array.
    
    This method is only available for users with NLib
***********************************************************************/
SmStatus SmBSplineSurface::CreateNSidedPatch
  (const SmContext                 & crContext,        // in : context for new object construction
   const SmTArray<SmBSplineCurve*> & crBdryCrvs,       // in : NON-RATIONAL  boundary curves; cur[0],...,cur[k]
                                                       //      must be  input  consecutively along  the n-sided
                                                       //      boundary and each curve parameterized in the same direction                                       
   const SmTArray<SmBSplineCurve*> & crDerivCrvs,      // in : NON-RATIONAL compatible corresponding cross-derivative curves.          
                                                       //      der=NULL:No curves specified, else der[i]=NULL:just der[i] not specified. 
                                                       //      specified der values approximated to eps angle in degrees                 
   double                            dAngTolDeg,       // in : MaxAngDeg allowed between NSidedPatch cross-derivative 
                                                       //      vectors and associated input crDerivCrvs values.                               
   SmTArray<SmSurface*>            & rSurfaces,        // out: (k+1) surfaces, interpolating boundary curves, and
                                                       //       approximating cross-boundary derivatives         
   SmPoint3d                       * pOptCenterPoint,  // in : Point on plane tangent to NSidedPatch in its center.
                                                       //      NULL to ignore, default:[NULL]
   SmVector3d                      * pOptCenterNormal) // in : Normal of plane tangent to NSidedPatch in its center.
                                                       //      NULL to ignore, default:[NULL]
{
  // locals
  NL_CURVE    **cur, **der;
  NL_INDEX    k;
  NL_REAL     eps = dAngTolDeg;
  NL_VECTOR   CI, NI;
  NL_FLAG     flg = NL_NO ;
  NL_SURFACE  **sur;
  NL_STACKS   SC, SS;

  // Allocate space for curves arrays
  SmNLibStackHandler sSH (&SC) ;
  SmNLibStackHandler sSH1(&SS) ;
  k = crBdryCrvs.GetSize()-1 ;
  cur = N_AllocArrayCrvPtrs(k,&SC) ; NER(cur);
  der = N_AllocArrayCrvPtrs(k,&SC) ; NER(der);

  // for every input pos/crossTan input pair - set NLib call input
  for (NL_INDEX i=0; i<=k; i++) 
    {
      cur[i] = crBdryCrvs[i]->GetOrCreateGwNurbPointer();
      if (crDerivCrvs[i] != NULL) 
        {
          der[i] = crDerivCrvs[i]->GetOrCreateGwNurbPointer();
        }
      else 
        {
          der[i] = NULL;
        }
    } // end iter

  // use input tangent plane - when given
  if(pOptCenterPoint && pOptCenterNormal)
    {
      flg = NL_YES ;
      CI.x = pOptCenterPoint->x ;
      CI.y = pOptCenterPoint->y ;
      CI.z = pOptCenterPoint->z ;

      pOptCenterNormal->Unitize() ;
      NI.x = pOptCenterNormal->x ;
      NI.y = pOptCenterNormal->y ;
      NI.z = pOptCenterNormal->z ;
    }

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  // draw 
  if(bDebugMe)
    {
      NL_INDEX ii ;
      smgfx_Erase() ;
      for(ii=0;ii<=k;ii++) { NL_CURVE *pCur = cur[ii] ;
                             NL_CURVE *pDer = der[ii] ;
                             Dump_NCrv(pCur) ;
                             if (pDer) Dump_NCrv(pDer) ;
                             if (crDerivCrvs[ii]) crDerivCrvs[ii]->Dump() ;
                             smgfx_SetLook(3,4, 0,0,1) ; smgfx_DrawNCrv(pCur) ; sm_GraphicsLoop() ;
                             if (pDer)
                               { smgfx_SetLook(1,2, 1,0,0) ; smgfx_DrawCrossTangentNCrv(pDer, pCur) ; sm_GraphicsLoop() ; }
                           }
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // pass the call along to NLib
  NL_SER(N_FillNSidedHole(cur,     // in : NON-RATIONAL  boundary curves; cur[0],...,cur[k]                         
                                   //      must be  input  consecutively along  the n-sided                         
                                   //      boundary                                                                 
                          der,     // in : NON-RATIONAL compatible corresponding cross-derivative curves.                        
                                   //      der=NULL:No curves specified, else der[i]=NULL:just der[i] not specified.
                                   //      specified der values approximated to eps angle in degrees                
                          k,       // in : Highest index in cur and der arrays                                      
                          eps,     // in :  max angDeg allowed between NSidedPatch bndryCrossDerivs and der values
                         &CI,      // in : CenterPoint of plane tangent to NSidedPatch at its center                                
                         &NI,      // in : UnitNormal  of plane tangent to NSidedPatch at its center                                
                          flg,     // in : NL_YES = CI and NI are passed in                                         
                                   //      NL_NO  = compute CI and NI internally                                    
                          &sur,    // out: (k+1) surfaces, interpolating boundary curves, and                         
                                   //       approximating cross-boundary derivatives                                
                          &SC,     // in : cur's and der's stack                                                    
                          &SS));   // in : sur's stack                                                              

  // for every gw_SURFACE surface - build an SmBSPlineSurface output object 
  for (NL_INDEX j=0; j<=k; j++) 
    {
      SmBSplineSurface * pSurface = new (crContext) SmBSplineSurface((gw_SURFACE *)sur[j]) ;
      rSurfaces.Add(pSurface);
    }
  
  // all done
  return SM_SUCCESS;

} // end SmBSplineSurface::CreateNSidedPatch

/*******************************************************************//**
PURPOSE: Create a ruled surface from two curves.

NOTES:
    Curve1 and Curve2 should not contain creases, otherwise an invalid surface
    will be created. Break each curve at g1 discontinuity and create a list
    of ruled surfaces or faces.

    crCurve1 and crCurve2 are not consumed nor owned by the new surface
***********************************************************************/
SmStatus SmBSplineSurface::CreateRuledSurface
  (const SmContext      & crContext,           // in : context for new object construction
   const SmBSplineCurve & crCurve1,            // in : Shape for Min U or V surface isoparam curve 
   const SmBSplineCurve & crCurve2,            // in : Shape for Max U or V surface isoparam curve 
   SmSurfParamType        eLinearSurfParam,    // in : SM_SP_U = Surface U dir is linear, input curves vary in V
                                               //      SM_SP_V = Surface V dir is linear, input curves vary in U
   SmBSplineSurface    *& rpNewBSplineSurface) // out: The new ruled surface 
{
  // reset output
  rpNewBSplineSurface = NULL;

  // locals
  NL_STACKS             nlStacks;
  SmNLibStackHandler sSH(&nlStacks);
  NL_CURVE              cur1, cur2;
  NL_SURFACE            sur;
  NL_FLAG            dir = (eLinearSurfParam == SM_SP_U) ? NL_UDIR : NL_VDIR;

  N_SrfInitArrays(&sur);

  // NLib curves from input curves
  SER(sm_BuildNlibCurve(((SmBSplineCurve *)&crCurve1),nlStacks,cur1));
  SER(sm_BuildNlibCurve(((SmBSplineCurve *)&crCurve2),nlStacks,cur2));

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  // draw 
  if(bDebugMe)
    {
      SM_ASSERT_VALID(&crCurve1) ;
      SM_ASSERT_VALID(&crCurve2) ;

      crCurve1.Dump() ;
      crCurve2.Dump() ;

      smgfx_Erase() ;
      smgfx_SetLook(2,3, 1,0,0) ; crCurve1.Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(2,3, 1,0,0) ; crCurve2.Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(2,3, 0,1,0) ; crCurve1.DrawWithKnots() ; sm_GraphicsLoop() ;
      smgfx_SetLook(2,3, 0,0,1) ; crCurve2.DrawWithKnots() ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE
              
  // pass the call along to NLib           
  NL_SER(N_CreateRuledSrf(&cur1,&cur2,dir,&sur,&nlStacks,&nlStacks));

  // BSplineSurface from NLib SURFACE
  rpNewBSplineSurface = new (crContext) SmBSplineSurface((gw_SURFACE *)&sur) ;

  // all done
  return SM_SUCCESS;

} // end SmBSplineSurface::CreateRuledSurface

/*******************************************************************//**
PURPOSE: Create a ruled surface from two curves. 
 
NOTES: Note that this one 
   will modify the input curves to make sure they create a nice surface.

   Curve1 and Curve2 should not contain creases, otherwise an invalid surface
    will be created. Break each curve at g1 discontinuity and create a list
    of ruled surfaces or faces.
***********************************************************************/
SmStatus SmBSplineSurface::CreateRuledSurfaceSafe
  (const SmContext   & crContext,             // in : context for new object construction
   SmBSplineCurve    & rCurve1,               // in : Shape for Min U or V surface isoparam curve
   SmBSplineCurve    & rCurve2,               // in : Shape for Max U or V surface isoparam curve
   SmSurfParamType     eLinearSurfParam,      // in : SM_SP_U = Surface U dir is linear, input curves vary in V
                                              //      SM_SP_V = Surface V dir is linear, input curves vary in U
   SmBSplineSurface *& rpNewBSplineSurface)   // out: The new ruled surface
{
  if (rCurve1.GetDegree() == 3) 
    { SER(rCurve1.DampenKnotSpacing(2.0,3)); }

  if (rCurve2.GetDegree() == 3)  
    {  SER(rCurve2.DampenKnotSpacing(2.0,3)); }

  if (rCurve1.GetDegree() > 2 || rCurve2.GetDegree() > 2) 
    {
      SmBSplineCurve * apData[3];
      SmTArray<SmBSplineCurve*> sCurves(3,apData);
      sCurves.Add(&rCurve1);
      sCurves.Add(&rCurve2);
      SER(SmBSplineCurve::MakeCurvesCompatible(sCurves));
    }

  // pass the call along
  SER(SmBSplineSurface::CreateRuledSurface(crContext,             // in : context for new object construction
                                           rCurve1,               // in : Shape for Min U or V surface isoparam curve
                                           rCurve2,               // in : Shape for Max U or V surface isoparam curve
                                           eLinearSurfParam,      // in : SM_SP_U = Surface U dir is linear, input curves vary in V
                                                                  //      SM_SP_V = Surface V dir is linear, input curves vary in U
                                           rpNewBSplineSurface)); // out: The new ruled surface

  // all done
    return SM_SUCCESS;

} // end SmBSplineSurface::CreateRuledSurfaceSafe

/* ---------------------------------------------------------------------


   DESCRIPTION:

     This fitting routine computes the knot vector for global curve  
     interpolation. A typical calling example is:

       REAL        *u;
       NL_INDEX       n;
       DEGREE      p;
       KNOTVECTOR  knt;
       ...
       (get array u, choose p and define knt);
       ...
       N_FitCrvCalcKnotVector(u,n,p,&knt);

     IT IS ASSUMED THAT MEMORY FOR knt IS  ALLOCATED IN THE CALLING
     ROUTINE.


   ACCESS:
   
     u   , input  ,  Parameters
     n   , input  ,  Highest index in u
     p   , input  ,  Degree of interpolating curve
     knt , output ,  Knot vector


   RETURN CODES:

     None

   ------------------------------------------------------------------ */
static void sm_MakeKnots
  (NL_REAL *u, 
   NL_INDEX n, 
   NL_DEGREE p, 
   NL_KNOTVECTOR *knt )
{
  NL_INDEX   i, j, k;
  NL_REAL    *U, sum, dSize, dBack, dForward;
  NL_STACKS  SL;

  /* Start NURBS */
  N_InitNurbs(&SL);

  /* Extract the knots */
  N_KnotVectorGetKnots(knt,&i,&U);

  /* Compute knot vector.
   *  Full multiplicity (p+1) on the ends,
   *  and Greville abscissae on the interior.
   */

  for( i=0; i<=p; i++ )
  {
    U[  i  ] = u[0];
    U[n+i+1] = u[n];
  }

  for( i=1; i<=n-p; i++ )
  {
      sum = 0.0;
      for( j=i; j<=i+p-1; j++ )  sum += u[j];
      U[i+p] = sum/p;
      /* GAC - added following loop to clamp U to knots */
      /* bd: not sure we want to do this, Greville abscissae are appropriate. */
      /*     But it's never hit in prog_test. */
      for (k=1; k<=n; k++) {
          if (U[i+p] < u[k]) {
              dSize = u[k]-u[k-1];
              dBack = U[i+p] - u[k-1];
              dForward = u[k] - U[i+p];
              double dTest = U[i+p];
              if (dBack < dSize/3.0) {
                  dTest = u[k-1];
              }
              else if (dForward < dSize/3.0) {
                  dTest = u[k];
              }
              // Try our best to avoid triple knots
              if (SM_ARE_SAME(dTest,U[i+p-1]) &&
                  SM_ARE_SAME(dTest,U[i+p-2])) {
                  dTest = (u[k]-u[k-1])/2.0;
              }
              U[i+p] = dTest;
              break;
          }
      }
  }

} // end sm_MakeKnots

/*******************************************************************//**
PURPOSE: Create a skinned surface cross section curves and optionally
    one or two rail curves.

NOTES: This method is only available for users with NLib

  If rail curves are provided, their degree must not be greater than
  the number of cross-section curves minus one.

  lDegree can not be greater than number of cross-section curves.

  Cross sections curves must all go in the same direction.

  If rails are provided, pRail1 is at the beginning of the section curves,
  and pRail2 is at their ends.

METHOD --- converts input data forms and passes call along to Nlib
***********************************************************************/
SmStatus SmBSplineSurface::CreateSkinnedSurface
  (const SmContext                  & crContext,             // in : context for new geometry construction
   const SmTArray<SmBSplineCurve *> &crCrossSectionCurves,   // in : CrossSectionCurves in order of occurrence on surface
   SmBoolean                         bSyncronized,           // in : TRUE    = CrossSectionCurves have same degree and are synchronized
   SmSurfParamType                   eCrossSectionSurfParam, // in : SM_SP_U/SM_SP_V = CrossSectionCurves become surface U/V isoparameter curves.
   double                            dThisApproxTol3d,       // in : 0.0 = skinning interpolates crossSections, else approximate.
   const SmBSplineCurve            * pRail1,                 // in : Min param sideBoundary curve of surface. Null to ignore.
   const SmBSplineCurve            * pRail2,                 // in : Max param sideBoundary curve of surface. Null to ignore.  
   SmBoolean                         bRail1UsedAsSpineCurve, // in : TRUE = use pRail1 as spine curve. (don't use pRail2)
                                                             //      Note: When both rails are specified they must be syncronized.
                                                             //      Note: CrossSections positioned along rails at pIntersectionsWithRails param values
   const SmTArray<double>          * pIntersectionsWithRails,// in : Rail params of CrossSectionCurve/Rail intersections.
   SmBSplineSurface                * pDerivSurf[2],          // in : pDerivSurf[0] = skinned surface start CrossBoundary derivatives. NULL to ignore.
                                                             //      pDerivSurf[1] = skinned surface end CrossBoundary derivatives. NULL to ignore.
                                                             //      not used when bRail1UsedAsSpineCurve == TRUE
   SmBSplineSurface               *& rpNewBSplineSurface,    // out: The skinned surface
   ULONG                             lDegree)                // in : sweep direction degree, default:[3]
{
  // locals
  ULONG lNumSections = crCrossSectionCurves.GetSize();
  SmTArray<void*> sCrossSections( lNumSections );
  NL_STACKS     nlStacks;
  SmNLibStackHandler sSH(&nlStacks);

  // check input - when using a spine, spine curve and IntersectionsWithRails should be given
  if (bRail1UsedAsSpineCurve) { NER(pRail1);
                                NER(pIntersectionsWithRails);
                              }

  if( (pRail1 || pRail2) && (pIntersectionsWithRails == NULL)) { SER(SM_ERR); }

  // We'll need a local non-const copy of this:
  SmTArray< double > sIntParams;
  SmTArray< double > *pIntParamsPtr = NULL;
  if ( pIntersectionsWithRails != NULL )
    {
      sIntParams    = *pIntersectionsWithRails;
      pIntParamsPtr = &sIntParams;
    }


  // When given two rails - synchronize the knot vectors
  if (pRail1 && pRail2) 
    {
      // Note, this will reparameterize the curves, which will render
      // the intersection parameters invalid.
      // We'll have to correct that.
      SmExtent1d sDomain1Before = pRail1->GetNaturalInterval();
      SmExtent1d sDomain2Before = pRail2->GetNaturalInterval();

      SM_PTR_ARRAY(sRails,SmBSplineCurve,16);
      sRails.Add(SM_CONST_CAST(SmBSplineCurve*,pRail1));
      sRails.Add(SM_CONST_CAST(SmBSplineCurve*,pRail2));
      SER(SmBSplineCurve::SyncronizeKnotsOfCurves(sRails,SM_EFF_ZERO * 100));

      // See about adjusting the intersection parameters.
      SmExtent1d sDomain1After = pRail1->GetNaturalInterval();
      SmExtent1d sDomain2After = pRail2->GetNaturalInterval();

      ULONG ii;
      if ( pIntersectionsWithRails != NULL ) // (really shouldn't be Null.)
        {
          SmBoolean bFound;
          double dFraction, dNewParam, dTRail, dTCross, dDev;
          double dTol = 0.001 * pRail1->ApproximateLength( sDomain1After, 6 );

          // Check rail 1.
          if ( smos_Fabs( sDomain1After.GetMin() - sDomain1Before.GetMin() ) > SM_EFF_ZERO
           ||  smos_Fabs( sDomain1After.GetMax() - sDomain1Before.GetMax() ) > SM_EFF_ZERO )
            {
              for ( ii = 0; ii < lNumSections; ii++ )
                {
                  // Map the parameter from the old domain to the new one.
                  sDomain1Before.Inversion( sIntParams[ii], dFraction );
                  dNewParam = sDomain1After.Evaluate( dFraction );

                  // This linear mapping should be pretty much exact, but let's
                  // reintersect anyway, to be safe.  Since we have a very good guess,
                  // it shouldn't take more than one iteration.
                  SmBSplineCurve *pSectCrv = crCrossSectionCurves[ii];
                  SmExtent1d sCrossDomain = pSectCrv->GetNaturalInterval();
                  pRail1->LocalCurveIntersect( sDomain1After,
                            *pSectCrv, sCrossDomain,
                            dTol, dNewParam, sCrossDomain.GetMin(),
                            bFound, dTRail, dTCross, dDev );
                  sIntParams[ii] = ( bFound ) ? dTRail : dNewParam;

                } // end for each cross section curve
            } // end if domains don't match

          // Check rail 2.
          if ( smos_Fabs( sDomain2After.GetMin() - sDomain2Before.GetMin() ) > SM_EFF_ZERO
           ||  smos_Fabs( sDomain2After.GetMax() - sDomain2Before.GetMax() ) > SM_EFF_ZERO )
            {
              for ( ii = 0; ii < lNumSections; ii++ )
                {
                  // Map the parameter from the old domain to the new one.
                  sDomain2Before.Inversion( sIntParams[lNumSections+ii], dFraction );
                  dNewParam = sDomain2After.Evaluate( dFraction );

                  // This linear mapping should be pretty much exact, but let's
                  // reintersect anyway, to be safe.  Since we have a very good guess,
                  // it shouldn't take more than one iteration.
                  SmBSplineCurve *pSectCrv = crCrossSectionCurves[ii];
                  SmExtent1d sCrossDomain = pSectCrv->GetNaturalInterval();
                  pRail2->LocalCurveIntersect( sDomain2After,
                            *pSectCrv, sCrossDomain,
                            dTol, dNewParam, sCrossDomain.GetMax(),
                            bFound, dTRail, dTCross, dDev );
                  sIntParams[lNumSections+ii] = ( bFound ) ? dTRail : dNewParam;

                } // end for each cross section curve
            } // end if domains don't match
        }
   } // end synchronizing knot vectors, if given both rails


  // load rails into rails array and save knot vector
  NL_CURVE * rails[2] = { NULL, NULL } ;
  SmTArray<double> sKnots;
  if (pRail2) 
    {
      pRail2->GetKnots(sKnots);
      rails[1] = sm_CreateNlibCurve((SmBSplineCurve *)pRail2,nlStacks);
    }
  if (pRail1) 
    {
      pRail1->GetKnots(sKnots);
      rails[0] = sm_CreateNlibCurve((SmBSplineCurve *)pRail1,nlStacks);
    }

  // If there are only a couple of profile curves,
  // the degree of the rails must be high enough to fit
  // the contraints, in the sweep direction.  [B542]
  if ( lNumSections <= 3 )
    {
      ULONG lNumDerivSurfs = 0;
      if ( pDerivSurf != NULL )
        {
          if ( pDerivSurf[0] != NULL ) { lNumDerivSurfs++; }
          if ( pDerivSurf[1] != NULL ) { lNumDerivSurfs++; }
        }
      ULONG lNumConstraints = lNumSections + lNumDerivSurfs;
      NL_DEGREE deg;
      if ( rails[0] != NULL )
        {
          N_CrvGetDegree( rails[0], &deg );
          if ( ((ULONG)deg+1) < lNumConstraints )
            {
              NL_INDEX iIncrement = lNumConstraints-1 - deg;
              N_CrvElevateDegree( rails[0], iIncrement, rails[0], &nlStacks, &nlStacks );
            }
        }
      if ( rails[1] != NULL )
        {
          N_CrvGetDegree( rails[1], &deg );
          if ( ((ULONG)deg+1) < lNumConstraints )
            {
              NL_INDEX iIncrement = lNumConstraints-1 - deg;
              N_CrvElevateDegree( rails[1], iIncrement, rails[1], &nlStacks, &nlStacks );
            }
        }
    }


  // init SURFACE structure
  NL_SURFACE nlSurf;
  N_SrfInitArrays(&nlSurf);

#ifdef SM_DEBUG_CODE
  TCHAR sBuff1[SM_TBLOCK_SIZE];
  SmBoolean bDebugMe     = FALSE;
  SmBoolean bDebugOutput1 = FALSE;
  if (bDebugMe) 
    {
      // dump, write and draw inputs - rail curves, crossSections and derivSrfs
      if(pIntersectionsWithRails) { SM_DUMP_TARRAY( (*pIntersectionsWithRails ) ); }
      if(pIntParamsPtr          ) { SM_DUMP_TARRAY( (*pIntParamsPtr           ) ); }
      if(pRail1)                  { pRail1->Dump() ; if(bDebugOutput1) pRail1->WriteToFile(_T("rail0.txt")); }
      if(pRail2)                  { pRail2->Dump() ; if(bDebugOutput1) pRail2->WriteToFile(_T("rail1.txt")); }
      for(ULONG di=0;di<crCrossSectionCurves.GetSize();di++)
        { crCrossSectionCurves[di]->Dump(); 
          if (bDebugOutput1) { smos_sprintf(sBuff1, _T("cross%d.txt"), (int)di);
                              crCrossSectionCurves[di]->WriteToFile(sBuff1);
                            }
        }
      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pRail1) pRail1->Draw(); sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,0) ; if(pRail2) pRail2->Draw(); sm_GraphicsLoop() ;

      if ( pDerivSurf != NULL ) {
          smgfx_SetLook(1,2, 1,0,1) ; if(pDerivSurf[0]) { pDerivSurf[0]->Draw(TRUE) ; } sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 1,0,1) ; if(pDerivSurf[1]) { pDerivSurf[1]->Draw(TRUE) ; } sm_GraphicsLoop() ;

          smgfx_SetLook(1,2, 1,0,1) ; if(pDerivSurf[0]) { pDerivSurf[0]->DrawAlong(SM_SP_V, 0.0, 1) ; } sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 1,0,1) ; if(pDerivSurf[1]) { pDerivSurf[1]->DrawAlong(SM_SP_V, 0.0, 1) ; } sm_GraphicsLoop() ;
      }

      smgfx_SetLook(3,4, 1,0,0); for(ULONG dj=0;dj<crCrossSectionCurves.GetSize();dj++)
                                   { crCrossSectionCurves[dj]->Draw(); sm_GraphicsLoop() ; }
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // iter every crossSection curve building the sCrossSections array
  for( ULONG ii=0; ii<lNumSections; ii++) 
    {
      NL_CURVE *pCur = sm_CreateNlibCurve(crCrossSectionCurves[ii],nlStacks);
      sCrossSections.Add(pCur);

    } // end iter every crossSection curve building the sCrossSections Array

  // synchronize cross section curves when needed
  //if(!bSyncronized)
  // {
  //    bSyncronized = TRUE ;

  //    NL_INDEX k = sCrossSections.GetSize() - 1;
  //    CURVE ** cur = (CURVE **)sCrossSections.GetDataArray();

      // Syncronize the curves with knot vectors
      //NL_SER(N_CrvsMakeCompatibleAdjKnots(cur,k,dTKnotTol,&nlStacks));
  //    NL_SER(N_CrvsMakeCompatibleKnotTol(cur,k,dTKnotTol,&nlStacks));
  //  }



  // load start and end skin surface Cross-boundary derivatives int pDirArray
  NL_SURFACE * nlDerivs[2] = { NULL, NULL } ;
  NL_SURFACE ** pDirArray = NULL;
  if (pDerivSurf != NULL) // this should not be a NULL pointer
    {
      for (ULONG jj = 0; jj < 2; jj++)
        {
          if ( pDerivSurf[jj] )
            {
              nlDerivs[jj] = sm_CreateNlibSurface( pDerivSurf[jj], nlStacks );
            }
        }
  
      if (pDerivSurf[0] || pDerivSurf[1])
        {
          pDirArray = &nlDerivs[0];
        }
    } // end given start and/or end cross-boundary derivative check

  // NLib locals
  NL_CURVE     ** nlCurv             = (NL_CURVE **)sCrossSections.GetDataArray();
  NL_PARAMETER  * nlIntParams        = pIntParamsPtr ? (NL_PARAMETER*)pIntParamsPtr->GetDataArray() : NULL ;
  NL_INDEX        nlNumCrossCurves = lNumSections;
  NL_FLAG      nlCont[2]          = { NL_G1, NL_G1 };
  NL_FLAG      nlBdry[2]          = { NL_BOTTOM, NL_BOTTOM }; // specify which isoparameter curve in pDerivSurfs contains derivative data
                                                        //   NL_LEFT   : nlCurv corresponds to u=umin boundary
                                                        //   NL_RIGHT  : nlCurv corresponds to u=umax boundary
                                                        //   NL_BOTTOM : nlCurv corresponds to v=vmin boundary
                                                        //   NL_TOP    : nlCurv corresponds to v=vmax boundary
  NL_FLAG         nlDir             = (eCrossSectionSurfParam == SM_SP_U) ? NL_UDIR : NL_VDIR ;
  NL_FLAG         nlSync            = (bSyncronized) ? NL_YES : NL_NO ;
  NL_CURVE     ** nlRailArray      = (pRail1 || pRail2) ? &rails[0] : NULL;
  NL_DEGREE       nlDeg               = (NL_DEGREE)lDegree;
  NL_KNOTVECTOR * nlKnots             = NULL;


  // If there are no additional constraints, then use the lowest degree
  // that will interpolate the given number of cross curves in a single span.
  // [5/28/10]
  if ( pDirArray == NULL ) // This would be the additional constraints.
    {
      if ( lDegree >= (ULONG)nlNumCrossCurves )
        { nlDeg = (NL_DEGREE)(nlNumCrossCurves - 1); }
    }

  // when pIntersectionsWithRails is given
  if (nlIntParams != NULL) 
    {
      // and knots are available from the rail curves
      if (sKnots.GetSize() > 0) 
        {
          // use the rail knot vector
          if      (rails[0]) { nlKnots = rails[0]->knt;} 
          else if (rails[1]) { nlKnots = rails[1]->knt; }
        }
      else // use a regularly spaced knot vector
        {
          nlKnots = N_AllocKnotVectorAndArray( nlNumCrossCurves+nlDeg, &nlStacks );
          sm_MakeKnots( nlIntParams, nlNumCrossCurves-1, nlDeg, nlKnots );
        }
    }
  else // pIntersectionsWithRails is not given
    {
      // don't specify a knot vector
      nlKnots = NULL;
    }
  
  // when cross-derivatives are not specified - don't specify a knot vector (why?)
  if (pDirArray == 0) 
    {
      nlKnots = NULL;
    }

  // when using a rail curve
  if ( bRail1UsedAsSpineCurve ) 
    {
      // sub = Subdivision level; the higher the sub, the more Z
      //          vectors are computed to  obtain a smoothly moving
      //          local frame  along the spine. If  k is large, sub
      //          should be small (1 or 2). If k is small, a larger
      //          sub, e.g. 5-10, is recommended.
      NL_INDEX sub = ( nlNumCrossCurves < 9 ) ? 5 : 2 ;

      SmVector3d sPV[2];
      SmVector3d sXAxis, sYAxis, sZAxis;

      // evaluate rail curve start and use the tangent to build a orthonormal vector set
      SER(pRail1->Evaluate(pRail1->GetNaturalInterval().GetMin(),1,TRUE,sPV));
      SER(sPV[1].MakeUnitOrthoVectors(NULL,sXAxis,sYAxis,sZAxis));

#ifdef SM_DEBUG_CODE
      if ( bDebugMe )
      {
          smgfx_SetLook( 4,6, 1,0,0 ); sZAxis.Draw( sPV ); sm_GraphicsLoop();
          sm_GraphicsLoop();
      }
#endif // SM_DEBUG_CODE

      NL_VECTOR  nlZAxis;
      COPY_XYZ(sZAxis,nlZAxis);

      // pass the call along to create Skin Spine
      NL_SER(N_CreateSkinSpine(nlCurv,                         // in : Cross-section curves
                               nlNumCrossCurves-1,             // in : highest index in cur array
                               !nlSync,                        // in : NL_NO = make curves compatible, NL_YES = curves are compatible
                               rails[0],                       // in : Spine Curve
                               nlIntParams,                    // in : Spine Parameters for Cross-section/Spine intersections
                               NL_YES,                         // in : NL_YES = align cross-section curves, NL_NO : cross-section curves already aligned
                               nlZAxis,                        // in : Start local z-axis of the spine
                               sub,                            // in : subdiviaion level: higher sub, the more z vectors are computed to smooth moving local frame
                               nlDeg < 2 ? NL_NODER : NL_ENDDER, // in : NL_NODER = no derivs, NL_ENDDER = fit derivs at end cross-sections, NL_ALLDER = fit derivs at every cross-section
                               nlDeg,                            // in : degree in skinning direction. MUST be 2 or 3 when NL_ALL_DER is specified
                               nlDir,                            // in : NL_UDIR = sking in U dir, NL_VDIR = skin in v direction
                               &nlSurf, &nlStacks, &nlStacks));  // out: skinned surface, cur's memory stack, sur's memory stack
    }

  // else skin crossSections with optional rail curves and start/end cross-derviative constraints
  else if (NL_YES == N_CreateSkinSrfBoundaryContinuity
                         (nlCurv,                // in : Cross-section curves
                          nlNumCrossCurves-1,    // in : highest index in cur array
                          pDirArray,             // in : HOMOGENEOUS  derivative  data  from  which  cross-
                                                 //      boundary continuity is computed:
                                                 //        nlDerivs[0]: derivative across cur[0]
                                                 //        nlDerivs[1]: derivative across cur[k]
                          nlSync,                // in : NL_NO = make curves compatible, NL_YES = curves are compatible
                          nlCont,                // in : nlCont[0]: NL_G1 or NL_C1 for nlDerivs[0]
                                                 //      nlCont[1]: NL_G1 or NL_C1 for nlDerivs[1]
                          nlBdry,                // in : nlBdry[0]: nlDerivs[0]'s type: NL_LEFT, NL_RIGHT, NL_BOTTOM or NL_TOP
                                                 //      nlBdry[1]: nlDerivs[1]'s type: NL_LEFT, NL_RIGHT, NL_BOTTOM or NL_TOP
                          nlRailArray,           // in : rail[0]: rail for v = vmin, rail[1]: rail for v = vmax, NULL to ignore
                          nlDeg,                 // in : degree in skinning direction
                          dThisApproxTol3d,      // in : 0.0 crossSections interpolated, else approximated
                          nlDir,                 // in : NL_UDIR: u-skin (cur[i] are v-curves), NL_VDIR: v-skin (cur[i] are u-curves)
                          nlIntParams,           // in : rail parameters for crossSection/Rail intersections
                          nlKnots,               // in : Knot vector in skinning direction, NULL = compute internally
                                                 //       rail != NULL: nlKnots must contain knots of rail
                                                 //       rail == NULL: nlKnots used for skinning
                          &nlSurf,&nlStacks,&nlStacks)) // out: skinned surface, cur's memory stack, nlSurf's memory stack
    {
      // If not successful try without knots.
      if (N_CreateSkinSrfBoundaryContinuity(nlCurv, 
                                            nlNumCrossCurves-1,
                                            pDirArray,
                                            nlSync,
                                            nlCont,
                                            nlBdry,
                                            nlRailArray,
                                            nlDeg,
                                            dThisApproxTol3d,
                                            nlDir,
                                            nlIntParams,
                                            NULL,                      // <=== different
                                            &nlSurf,&nlStacks,&nlStacks) == NL_YES) 
        {
          // It failed with derivative fields try it without pDirArray
          if (pDirArray != NULL) 
            {
              NL_SER(N_CreateSkinSrfBoundaryContinuity(nlCurv, 
                                                       nlNumCrossCurves-1,
                                                       NULL,              // <=== different
                                                       nlSync,
                                                       nlCont,
                                                       nlBdry,
                                                       nlRailArray,
                                                       nlDeg,
                                                       dThisApproxTol3d,
                                                       nlDir,
                                                       nlIntParams,
                                                       NULL,              // <=== different
                                                       &nlSurf,&nlStacks,&nlStacks));
            }
          else // its not working - signal an error
            {
              SER(SM_ERR);
            } // end CreateSkinSrfBoundaryContinuity without cross derivative and knot vector try branch
        }  // end CreateSkinSrfBoundaryContinuity    without knot vector try branch
    } // end CreateSkinSrfBoundaryContinuity         with    all inputs try branch

  // set output
  rpNewBSplineSurface = new (crContext) SmBSplineSurface((gw_SURFACE *)&nlSurf) ;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugCheckSurface = FALSE;
  if (bDebugCheckSurface)
    {
      // Check for knot validity of surface
      SmValidityCheckType eValidityCheckThatFailed;
      if ( ! rpNewBSplineSurface->PassesValidityCheck(SM_VC_ALL,eValidityCheckThatFailed)) 
        { SE(SM_ERR_INVALID_INPUT); }
    }
#endif // SM_DEBUG_CODE

#ifdef SM_DEBUG_CODE
  if (bDebugMe) 
    {
      SmBoolean bDebugOutput2 = FALSE;
      rpNewBSplineSurface->Dump();
      if (bDebugOutput2)
        { rpNewBSplineSurface->WriteToFile(_T("surf.txt")); }

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pRail1) pRail1->Draw(); sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,0) ; if(pRail2) pRail2->Draw(); sm_GraphicsLoop() ;

      if ( pDerivSurf != NULL ) {
          smgfx_SetLook(1,2, 1,0,1) ; if(pDerivSurf[0]) { pDerivSurf[0]->Draw(TRUE) ; } sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 1,0,1) ; if(pDerivSurf[1]) { pDerivSurf[1]->Draw(TRUE) ; } sm_GraphicsLoop() ;
    
          smgfx_SetLook(1,2, 1,0,1) ; if(pDerivSurf[0]) { pDerivSurf[0]->DrawAlong(SM_SP_V, 0.0, 1) ; } sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 1,0,1) ; if(pDerivSurf[1]) { pDerivSurf[1]->DrawAlong(SM_SP_V, 0.0, 1) ; } sm_GraphicsLoop() ;
      }

      smgfx_SetLook(3,4, 1,0,0); for(ULONG di=0;di<crCrossSectionCurves.GetSize();di++)
                                   { crCrossSectionCurves[di]->Draw(); sm_GraphicsLoop() ; }
      sm_GraphicsLoop();

      // draw output
      smgfx_SetLook(1,2, 0,1,1); rpNewBSplineSurface->DrawUV(1,1); sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,1); rpNewBSplineSurface->DrawNet(); sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,1); rpNewBSplineSurface->DrawPolygon(); sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 1,0,0) ; rpNewBSplineSurface->DrawAlong(SM_SP_U, 0.0, 1) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 1,0,0) ; rpNewBSplineSurface->DrawAlong(SM_SP_U, 1.0, 1) ; sm_GraphicsLoop() ;
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // all done
  return SM_SUCCESS;

} // end SmBSplineSurface::CreateSkinnedSurface

/*******************************************************************//**
PURPOSE: The routine creates a general sweep surface given a trajectory
    curve and a section curve to be swept along the trajectory. 
    
NOTES:
      It will create one of two types of swept surface: 
    (1) translational sweep surface given a section (u-)curve and a trajectory (v-)curve, and
    (2) sweep with boundary conditions.

    If bDoTranslationalSweep == FALSE, cross-boundary tangent directions
    in the sweep direction match those of the trajectory curve. Then the swept
    surface is computed by positioning scaled instances of the section curve
    along the trajectory, and skinning across them. The output Nurbs surface 
    attempts to approximate the true swept surface to within the given 
    tolerance. 

    The Scale curve is a curve which defines a scale of the section curve
    as it moves along the trajectory curve.  The scale curve defines the X,
    Y, Z scale of the cross section curve.  It normally is set up to be the
    same scale in X, Y, and Z.  The parameterization should be equal to the
    trajectory curve parameterization.

    This method is only available for users with NLib
    
***********************************************************************/
SmStatus SmBSplineSurface::CreateSweepSurface
  (const SmContext      & crContext,               // in : context for new object construction 
   const SmBSplineCurve * cpSectionCurve,          // in : u direction curve for new Surface
   const SmBSplineCurve * cpTrajectoryCurve,       // in : v direction curve for new Surface
   const SmBSplineCurve * cpOptScaleCurve,         // in : x,y,z scale values for SectionCurve for each v value
                                                   //      ScaleCurve->NaturalInterval must equal TrajectoryCurve->NaturalInterval
   SmBoolean              bDoTranslationalSweep,   // in : TRUE = NewSurf(u,v) = SectionCurve(u) + TrajectoryCurve(v)
                                                   //               this is an exact surface.
                                                   //               (cpOptScaleCurve and dThisApproxTol3d not used)
                                                   //      FALSE= NewSurf(u,v) =   A(v) * ScaleCurve(v) * SectionCurve(u) 
                                                   //                            + TrajectoryCurve(v)
                                                   //               this is an approximate surface.
   double                 dThisApproxTol3d,        // in : Max distance between exact surface and newSurf approximation 
   SmBSplineSurface    *& rpNewSurf)               // out: new sweptSurface
{
  // check input
  SM_ASSERT_MSG(rpNewSurf == NULL,_T("SmBSplineSurface::CreateSweepSurface - output pointer not NULL on input")) ;

  // locals
  NL_STACKS  nlStacks;
  NL_SURFACE nlSurf;
  NL_CURVE  *nlScaleCurv = NULL;

  // initialize GW NURB curve state
  SmNLibStackHandler sSH1(&nlStacks);

  // initialize surface structure to NULL
  N_SrfInitArrays(&nlSurf);

  // get NurbCurve pointers for Section and trajectory curves
  NL_CURVE *nlCurvS = ((SmBSplineCurve *)cpSectionCurve)->GetOrCreateGwNurbPointer();
  NL_CURVE *nlCurvT = ((SmBSplineCurve *)cpTrajectoryCurve)->GetOrCreateGwNurbPointer();
                
  // when given a scaling curve - get its NurbCurve Pointer
  if (cpOptScaleCurve) 
    {
      nlScaleCurv = ((SmBSplineCurve *)cpOptScaleCurve)->GetOrCreateGwNurbPointer();
    }

  // branch on TranslationalSweep flag
  if (bDoTranslationalSweep) 
    {
      // nlSurf(u,v) = nlCurvS(u) + nlCurvT(v) ;
      NL_SER(N_CreateTransSweepSrf( nlCurvT, nlCurvS, &nlSurf, &nlStacks ));
    }
  else 
    {
      // 

      // make orthogonal vector set
      // x = TrajectoryCurve Start Tangent
      // y = any perp vector to x
      // z = x * y
      SmVector3d sPV[2];
      SER(cpTrajectoryCurve->Evaluate(cpTrajectoryCurve->GetNaturalInterval().GetMin(),1,TRUE,sPV));
      SmVector3d sXAxis, sYAxis, sZAxis;
      SER(sPV[1].MakeUnitOrthoVectors(NULL,sXAxis,sYAxis,sZAxis));
      NL_VECTOR  nlZAxis;
      COPY_XYZ(sZAxis,nlZAxis);

      // degree would normally be cubic, except if trajectory = linear
      // and there is no scaling curve
      NL_DEGREE nlDeg = 3;
      if (nlCurvT->p < nlDeg )
        { nlDeg = nlCurvT->p; }
      if (nlScaleCurv && nlScaleCurv->p > nlDeg)
        { nlDeg = 3; }

      NL_PARAMETER nlDummy = 0.0;
      NL_SER(N_CreateSweepScale( nlCurvT, nlCurvS,
                                 nlZAxis,
                                 NULL,
                                 nlScaleCurv,
                                 NL_NO,
                                 nlDummy,
                                 nlDeg,
                                 NL_UDIR,
                                 dThisApproxTol3d,
                                 &nlSurf,
                                 &nlStacks));
    }

  // build SmBSplineSurface from NurbSurface
  rpNewSurf = new (crContext) SmBSplineSurface((gw_SURFACE *)&nlSurf) ;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  if (bDebugMe) 
    {
      sm_GraphicsLoop();
      smgfx_SetColor(0,0,1);
      rpNewSurf->DrawUV(1,1);
      rpNewSurf->Dump();
    }
#endif

  // all done
  return SM_SUCCESS;

} // end SmBSplineSurface::CreateSweepSurface


/*******************************************************************//**
   Local helper routines for CreateSweepSurface(), multi-profile version:
***********************************************************************/

/*******************************************************************//**
PURPOSE: Calculate a coordinate system for a position along two curves.

NOTES: 
   The X axis is from the first curve point to the second,
   and the Y axis is the average direction of the curves.
***********************************************************************/
static SmStatus sm_CalcLocalCS( SmCurve *pRail1, SmCurve *pRail2, double dParam,
                    SmAxis2Placement &rCS, double & rdXScale )
{
  // (Maybe the caller should eval the rails.)
  SmVector3d sPtVec1[2], sPtVec2[2];
  pRail1->Evaluate( dParam, 1, FALSE, sPtVec1 );
  pRail2->Evaluate( dParam, 1, FALSE, sPtVec2 );
  SmVector3d sXAxis( sPtVec2[0] - sPtVec1[0] );
  rdXScale = sXAxis.Length();
  sXAxis /= rdXScale;

  sPtVec1[1].Unitize();
  sPtVec2[1].Unitize();
  SmVector3d sYAxis = 0.5 * ( sPtVec1[1] + sPtVec2[1] );
  sYAxis.Unitize();

  SmStatus eStat = rCS.SetCanonical( sPtVec1[0], sXAxis, sYAxis );

  return eStat;
}

/*******************************************************************//**
PURPOSE: Get the control points of a curve, transformed by the given transform.

NOTES: 
   Also divide the X component by the given scale.
***********************************************************************/
static SmStatus sm_GetCtrlPtsInCS( const SmBSplineCurve* pCrv,
                    SmAxis2Placement & rCS, double dXScale,
                    SmTArray< SmPoint3d > & rProfilePts )
{
  SmTArray< double > sWts;
  pCrv->GetControlPolygon( rProfilePts, sWts );
  ULONG ii, lNumPts = rProfilePts.GetSize();

  for ( ii = 0; ii < lNumPts; ii++ )
  {
      rCS.InvTransformPoint( rProfilePts[ii], rProfilePts[ii] );
      rProfilePts[ii].x /= dXScale;
  }
  return SM_SUCCESS;
}

/*******************************************************************//**
PURPOSE: This routine creates a Sweep surface interpolating multiple
    cross-sectional curves along two guide curves.

NOTES: 
    The input mesh curves must fully intersect, and the parameter values at
    the intersections must match fairly closely along each row and column
    of the input curves.  As a corollary to that, each set of curves must
    be parameterized in the same direction.

    Provided the mesh of curves truly intersect at every row,column point
    in the mesh, then this routine will try to modify the curves slightly
    to make them parametrically acceptable for the surface construction.
    However, you may have problems if the curves in the rows or columns
    are very different in properties.

    If the input curves are rational, they will be approximated
    with nonrational curves (copies) for the surface creation.

    The surface will be created with the u parameter running along
    the rails, from profile to profile, and v running from rail to rail.
    So the first rail will be the v = 0 isocurve.

    This method is only available for users with NLib

Arguments:
  dTol : tolerance used for: approximating rational curves with nonrational,
    checking closure, and intersecting the curves.
    If not specified, we base it on curve lengths.

  lNumBetweenProfiless: minimum number of control points on the rails between
    each pair of profiles.  A row of control points will be created in the
    surface for each control point on the rails.  If there are none, then
    the surface can have creases at the profiles.
    Optional argument, default 4.

  lBlendLevel: Between the profiles, the surface 'profiles' are interpolations
    of the two surrounding given profiles.  Interpolating linearly can
    result in creases at the profiles.  Higher-order blending makes it smoother.
    Values are: 0: linear; 1: cubic; 2: quintic; 3: 7th degree.
    Optional argument, default 2, which looks best on test cases.

POSSIBLE ENHANCEMENTS ---
   1. This could be made to work with only a single guide curve.
      Currently, you could just put a curve through the end points
      of the section curves.
   2. This could possibly be made to work with more than two rails.
      Interior rails would have to interpolate the section curves
      in their interiors.
   3. It should be possible to loosen the restriction that the curves
      must all intersect.  For example, perhaps the rails could extend
      beyond the profiles, i.e., not have a profile at the end of the rails.
      The surface would end at the curve intersections.
***********************************************************************/
SmStatus SmBSplineSurface::CreateSweepSurface
( 
  const SmContext &crContext,                     // in :
  const SmTArray< SmBSplineCurve* > & crProfiles, // in :
  const SmTArray< SmBSplineCurve* > & crRails,    // in :
  double dTol,                                    // in : optional.  See notes.
  SmBSplineSurface * & rpNewSurf,                 // out:
  ULONG lNumBetweenProfiles,                      // in : optional.  See notes.
  ULONG lBlendLevel )                             // in : optional.  See notes.
{
  rpNewSurf = NULL;

  ULONG lNumProfs  = crProfiles.GetSize();
  if ( lNumProfs < 2 ) { return SM_ERR_INVALID_INPUT; }
  ULONG lNumRails = crRails.GetSize();
  if ( lNumRails < 2 ) { return SM_ERR_INVALID_INPUT; }

  NL_STACKS nlStacks;
  SmNLibStackHandler sSH(&nlStacks);

#define MAX_CURVES 640
  NL_CURVE *nlCurU[ MAX_CURVES ];
  NL_CURVE *nlCurV[ MAX_CURVES ];
#undef  MAX_CURVES

  NL_PARAMETER *nlParamsU, *nlParamsV;

  SmStatus eStat = SmBSplineSurface::PrepCurvesForSurfacing( crContext,
      crRails, // in
      crProfiles,  // in
      nlCurU,    // out
      nlCurV,    // out
      nlStacks,
      dTol,
      TRUE,    // MakeNonRational: necessary for this.
      lNumBetweenProfiles, // num cpts between profiles: otherwise we get creases at profiles.
      0,     // num cpts between rails: don't care.
      TRUE,  // common param ranges in each direction
      TRUE,  // common parameterization in each direction
      &nlParamsU, &nlParamsV,
      NULL, NULL // don't need all intersection params.
  );

  if ( eStat != SM_SUCCESS )
    { return eStat; }

  ULONG lUCount = crRails.GetSize();
  ULONG lVCount = crProfiles.GetSize();

  // Create SmBSplineCurves out of the prepared NLib curves.
  // Also copy params into SmTArrays.
  SmTArray< SmBSplineCurve* > sProfs, sRails;
  SmTArray< double > sRailParams, sProfParams;
  SmBSplineCurve *pTmpBSC;
  ULONG ii;
  for ( ii = 0; ii < lVCount; ii++ )
    {
      sRailParams.Add( nlParamsU[ii] );

      pTmpBSC = new(crContext) SmBSplineCurve( 3, (gw_CURVE *)nlCurV[ii] );
      sProfs.Add( pTmpBSC );
    }
  for ( ii = 0; ii < lUCount; ii++ )
    {
      sProfParams.Add( nlParamsV[ii] );

      pTmpBSC = new(crContext) SmBSplineCurve( 3, (gw_CURVE *)nlCurU[ii] );
      sRails.Add( pTmpBSC );
    }
  SmObjsDelete<SmBSplineCurve*> sCleanProfs( &sProfs );
  SmObjsDelete<SmBSplineCurve*> sCleanRails( &sRails );

  SmTArray< SmPoint3d > sProfPts0, sProfPts1, sThisProfPts;
  SmTArray< SmPoint3d > sSurfPts;

  SmAxis2Placement sCS0, sCS1, sThisCS;
  double dXScale0, dXScale1, dThisXScale;
  SmPoint3d sTempPt;

  // We're going to have to map back and forth between parameter values and
  // control point indices -- Greville abscissae.  Just collect them all.
  SmTArray< double > sRailGrevilles;
  SmTArray< double > sRailKnots;
  sRails[0]->GetKnotsAll( sRailKnots );
  ULONG lNumCpts = sRails[0]->GetNumberControlPoints();
  ULONG lRailDeg = sRails[0]->GetDegree();
  double dGrev;
  for ( ii = 0; ii < lNumCpts; ii++ )
  {
      SER( smgu_GetGrevilleAbscissa( ii, sRailKnots, lRailDeg, dGrev ));
      sRailGrevilles.Add( dGrev );
  }

  ULONG lIdx0, lIdx1 = 0;
  ULONG lNumProfPts  = sProfs[0]->GetNumberControlPoints();
  ULONG lNumRailPts  = sRails[0]->GetNumberControlPoints();

  SmTArray< double > sProfKnots;
  sProfs[0]->GetKnotsAll( sProfKnots );
  SmTArray< double > sProfGrevilles;
  lNumCpts = sProfs[0]->GetNumberControlPoints();
  ULONG lProfDeg = sProfs[0]->GetDegree();
  for ( ii = 0; ii < lNumCpts; ii++ )
  {
      SER( smgu_GetGrevilleAbscissa( ii, sProfKnots, lProfDeg, dGrev ));
      sProfGrevilles.Add( dGrev );
  }


  // Loop indices: we loop over each span, and within that, over each
  // control point along the way.
  // In order to correctly install the control points into the two-dimensional
  // array for surface construction, we have to run all the way along each profile
  // from beginning to end.  So the loop has to be:
  //   for each profile span
  //     for each rail-control-point in that profile span
  //       for each rail span
  //         for each profile-control-point in that rail span
  //
  ULONG lProfSpan, lRailCtrlPt, lRailSpan, lProfCtrlPt;

  for ( lProfSpan = 1; lProfSpan < lNumProfs; lProfSpan++ )
  {
      SmBSplineCurve *pProf0 = sProfs[lProfSpan-1];
      SmBSplineCurve *pProf1 = sProfs[lProfSpan  ];
      double dT0 = sRailParams[lProfSpan-1];
      double dT1 = sRailParams[lProfSpan  ];
      double dDeltaT = dT1 - dT0;
      if ( dDeltaT < SM_EFF_ZERO )
        { SER( SM_ERR_INVALID_INPUT ); }


      // For each control point pair on the rails, between dT0 and dT1:
      // so first we have to get the control point indices
      lIdx0 = ( lProfSpan == 1 ) ? 0 : lIdx1 + 1;

      while ( sRailGrevilles[ lIdx1 ] < dT1 && lIdx1 < lNumRailPts-1 )
        { lIdx1++; }

      for ( lRailCtrlPt = lIdx0; lRailCtrlPt <= lIdx1; lRailCtrlPt++ )
      {
          double dRailT = sRailGrevilles[ lRailCtrlPt ];

          ULONG lProfIdx1 = 0;
          for ( lRailSpan = 1; lRailSpan < lNumRails; lRailSpan++ )
          {
              SmBSplineCurve *pRail0 = sRails[lRailSpan-1];
              SmBSplineCurve *pRail1 = sRails[lRailSpan];

              SER( sm_CalcLocalCS( pRail0, pRail1, dT0, sCS0, dXScale0 ));
              SER( sm_CalcLocalCS( pRail0, pRail1, dT1, sCS1, dXScale1 ));

              sm_GetCtrlPtsInCS( pProf0, sCS0, dXScale0, sProfPts0 );
              sm_GetCtrlPtsInCS( pProf1, sCS1, dXScale1, sProfPts1 );
              if ( sProfPts1.GetSize() != lNumProfPts )
                { SER( SM_ERR_INVALID_INPUT ); }

              SER( sm_CalcLocalCS( pRail0, pRail1, dRailT, sThisCS, dThisXScale ));

              // Interpolate between sProfPts0 and sProfPts1.
              double dFrac = ( dRailT - dT0 ) / dDeltaT;

              // Don't just interpolate linearly, use some sort of sigmoid function.
              // This ensures smoothness in the rail direction,
              // when crossing profiles, as long as the rails are smooth.
              // Linear blending (use dFrac and 1-Frac as is) can leave creases.
              // Of degree 3, 5, or 7 blending, degree 5 seems to look the best,
              // so that's the default argument.

              // The functions are those that map function values 0 at 0
              // and 1 at 1 (to interpolate the profiles), and 0, 1, 2, or 3
              // zero derivatives at 0 and 1 (which ensures smoothness).
              // Cubic   :  -2x^3 +  3x^2
              // Quintic :   6x^5 - 15x^4 + 10x^3
              // 7th deg : -20x^7 + 70x^6 - 84x^5 + 35x^4

              if ( lBlendLevel == 1 ) // Cubic.
              {
                  dFrac = 3.0*dFrac*dFrac - 2.0*dFrac*dFrac*dFrac;
              }
              else if ( lBlendLevel == 2 ) // Quintic.
              {
                  double dd = dFrac * dFrac * dFrac;
                  dFrac = dd * ( 6.0*dFrac*dFrac - 15.0*dFrac + 10 );
              }
              else if ( lBlendLevel == 3 ) // Degree 7.
              {
                  double dd = dFrac * dFrac * dFrac * dFrac;
                  dFrac = dd * ( -20.0*dFrac*dFrac*dFrac + 70.0*dFrac*dFrac - 84.0*dFrac + 35 );

              }

              SmTArray< SmPoint3d > sPts;

              // We have to do the same trick here, with the profile Grevilles.
              ULONG lProfIdx0 = ( lRailSpan == 1 ) ? 0 : lProfIdx1 + 1;

              while ( sProfGrevilles[ lProfIdx1 ] < sProfParams[lRailSpan] && lProfIdx1 < lNumProfPts-1 )
                { lProfIdx1++; }

              for ( lProfCtrlPt = lProfIdx0; lProfCtrlPt <= lProfIdx1; lProfCtrlPt++ )
              {
                  sTempPt = (1-dFrac) * sProfPts0[lProfCtrlPt] + dFrac * sProfPts1[lProfCtrlPt];

                  // Transform back to global space and add to surface.
                  sTempPt.x *= dThisXScale;
                  sThisCS.TransformPoint( sTempPt, sTempPt );

                  sPts.Add( sTempPt );

              } // end loop on lProfCtrlPt, each ctrl pt on the profiles in this rail span.

              sSurfPts.Append( sPts );

          } // end for lRailSpan, loop on the spans between rails for this newly-created profile.

      } // end for lRailCtrlPt, on each ctrl pt on the rails between these two profiles.

  } // end for lProfSpan, on each span between profiles

  // Finally -- create the surface.
  // Get these again, in the form for CreateCanonical:
  SmTArray< ULONG  > sRailMults, sProfMults;
  sRails[0]->GetKnots( sRailKnots, &sRailMults );
  sProfs[0]->GetKnots( sProfKnots, &sProfMults );

  eStat = SmBSplineSurface::CreateCanonical( crContext, lRailDeg, lProfDeg,
      sSurfPts, SM_SF_UNSPECIFIED, sRailMults, sProfMults, sRailKnots, sProfKnots,
      SM_KT_UNSPECIFIED, NULL, NULL,
      rpNewSurf );

  return eStat;

} // end CreateSweepSurface

/*******************************************************************//**
PURPOSE: Create a swung surface given a profile curve in the [x,z] plane
    and a trajectory curve in the [x,y] plane. 

NOTES: This advance surface construction routine creates a swung surface
     given a profile curve in the  [x,z] plane  and a trajectory curve 
     in the [x,y] plane.  The swung surface is a generalization of the 
     surface of rotation in which the circular sweep curve is replaced 
     by a general sweep curve.
     
     This method is only available for users with NLib
***********************************************************************/
SmStatus SmBSplineSurface::CreateSwungSurface
(
  const SmContext      & crContext,          // in :
  const SmBSplineCurve * cpSectionCurve,     // in : XZ plane
  const SmBSplineCurve * cpTrajectoryCurves, // in : XY Plane
  double dScaleFactor,                       // out:
  SmBSplineSurface *& rpNewSurf              // out:
)       
{
    NL_CURVE   *nlCurvP, *nlCurvT;
    NL_SURFACE nlSurf;
    NL_STACKS  nlStacks;

    SmNLibStackHandler sSH1(&nlStacks);
    N_SrfInitArrays(&nlSurf);
    nlCurvP = ((SmBSplineCurve *)cpSectionCurve)->GetOrCreateGwNurbPointer();
    nlCurvT = ((SmBSplineCurve *)cpTrajectoryCurves)->GetOrCreateGwNurbPointer();
    NL_SER(N_CreateSwungSrf(nlCurvP,nlCurvT,dScaleFactor,&nlSurf,&nlStacks));

    rpNewSurf = new (crContext) SmBSplineSurface((gw_SURFACE *)&nlSurf) ;
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
    if (bDebugMe) {
        smgfx_Erase();
        smgfx_SetColor(1,0,0);
        cpSectionCurve->Dump();
        cpSectionCurve->Draw();
        sm_GraphicsLoop();
        smgfx_SetColor(0,0,1);
        cpTrajectoryCurves->Dump();
        cpTrajectoryCurves->Draw();
        sm_GraphicsLoop();
        smgfx_SetColor(0,0,1);
        rpNewSurf->DrawUV(1,1);
        rpNewSurf->Dump();
    }
#endif

    return SM_SUCCESS;

} // end SmBSplineSurface::CreateSwungSurface

/*******************************************************************//**
PURPOSE: Elevate the degree of a NURB surface in either U or V 
   by specified increment (1,2,..) without changing the geometry

NOTES: This method actually elevates the degree only if NLib is present.
***********************************************************************/
SmStatus SmBSplineSurface::DegreeElevate
(
  SmSurfParamType eDirectionToElevate,  // in : u- or v-direction
  ULONG lNewDegree                      // in :
)
{
    NL_STACKS nlStacks;
    SmNLibStackHandler sSH(&nlStacks);
    NL_SURFACE sur;
    N_SrfInitArrays(&sur);

    NL_SURFACE * surP = GetOrCreateGwNurbPointer() ;
    NER(surP);

    NL_FLAG UorV = 1;
    if (eDirectionToElevate == SM_SP_V) {
        UorV = 2;
    }
    NL_INDEX increm = lNewDegree - GetDegree(eDirectionToElevate);

    NL_SER(N_SrfElevateDegree(surP, increm, UorV, &sur, &nlStacks, &nlStacks));

    const SmContext * pContext = GetContext();
    NER(pContext);

    // place output sur in temporary pTemp
    SmBSplineSurface *pTemp = new (*pContext) SmBSplineSurface((gw_SURFACE *)&sur) ;
    SmObjDelete sClean(pTemp);

    // swap ptemp and this m_pNurb pointers
    gw_SURFACE * pNewSur = pTemp->GetOrCreateGwNurbPointer();
    pTemp->m_pNurb       = SM_REINTERPRET_CAST(gw_SURFACE*,surP) ;
    m_pNurb              =  pNewSur;

    return SM_SUCCESS;

} // end SmBSplineSurface::DegreeElevate


/*******************************************************************//**
PURPOSE: Reduce the degree of a NURB surface as much as possible 
   without changing the geometry more than the given tolerance.

NOTES: This method actually reduces the degree only if NLib is
   present.
***********************************************************************/
SmStatus SmBSplineSurface::DegreeReduction
(
  double dTolerance,                   // in :
  SmBoolean bMaxReduce,                // in : If TRUE, will reduce the degree as much as possible (both u- & v-dirs)
  SmSurfParamType eDirectionToReduce   // in : Specify u- or v-direction if reduce by only 1 degree
)
{
    NL_STACKS     nlStacks;
    SmNLibStackHandler sSH(&nlStacks);
    NL_SURFACE sur;
    N_SrfInitArrays(&sur);

    NL_SURFACE * surP = GetOrCreateGwNurbPointer();
    NER(surP);

    if (bMaxReduce) {
        // always outputs sur (even if only a copy, with no actual reduction)
        NL_SER(N_SrfReduceDegreeToTol(surP,dTolerance,&sur,&nlStacks,&nlStacks));
    }
    else {
        NL_FLAG dir = NL_UDIR;
        if (eDirectionToReduce == SM_SP_V) {
            dir = NL_VDIR;
        }
        NL_FLAG  rfl;
        NL_REAL  mtol;
        NL_SER(N_SrfReduceDegree(surP,dTolerance,dir,&rfl,&sur,&mtol,&nlStacks,&nlStacks));
        if (rfl == NL_NO) {
            //MSG(_T("Degree reduction failure"));
            return SM_SUCCESS;
        }
    }

    const SmContext * pContext = GetContext();
    NER(pContext);

    SmBSplineSurface *pTemp = new (*pContext) SmBSplineSurface((gw_SURFACE *)&sur) ;
    SmObjDelete sClean(pTemp);

    gw_SURFACE * pNewSur = pTemp->GetOrCreateGwNurbPointer();
    pTemp->m_pNurb       = surP ;
    m_pNurb              = pNewSur;

    return SM_SUCCESS;

} // end SmBSplineSurface::DegreeReduction

/*******************************************************************//**
PURPOSE: Evaluate point on the extended surface domain

NOTES: This method is only available for users with NLib
    
***********************************************************************/
SmStatus SmBSplineSurface::EvaluatePointOnExtendedSurface
(
  const SmPoint2d & crUV, // in :
  SmPoint3d & rPnt        // out:
) const
{
  // check input
#ifdef SM_DEBUG_CODE
//static const SmBSplineSurface *pLast = NULL ;
//        SM_ASSERT_MSG((pLast == this || IsBounded()), 
//                       _T("Err: Using infinite surface ControlPoints to make evaluation")) ;
//  if(!IsBounded()) { pLast = this ; }
#endif                  

    NL_SURFACE    esur;
    NL_SURFACE    *sur = ((SmBSplineSurface *)this)->GetOrCreateGwNurbPointer();
    NL_FLAG       gflg, cflg;
    NL_POINT       P;
    NL_STACKS     nlStacks;
    SmNLibStackHandler sSH(&nlStacks);
    N_InitNurbs(&nlStacks);
    cflg = NL_G1R;
    N_SrfInitArrays(&esur);
    NL_FLAG err = N_SrfEvalPtDerivsUnbounded(sur,crUV.x,crUV.y,0,cflg,&esur,&gflg,&P,NULL,&nlStacks);           
    if (err) {
        err = N_SrfEvalPtDerivsUnbounded(sur,crUV.x,crUV.y,0,cflg,&esur,&gflg,&P,NULL,&nlStacks);
        if (err) {
            SER(SM_ERR);
        }
    }
    COPY_XYZ(P,rPnt);

    N_EndNurbs(&nlStacks);

    return SM_SUCCESS;

} // end SmBSplineSurface::EvaluatePointOnExtendedSurface


/*******************************************************************//**
PURPOSE: Estimate average length of a surface in both u- and v-directions,
    and it computes a bound on surface area by taking the product of the
    maximums of length in both u- and v-directions. 

NOTES: This method actually reduces the degree only if NLib is
   present.
***********************************************************************/
SmStatus SmBSplineSurface::GetMeasures
(
  double & rdAverageLengthU,       // out:
  double & rdAverageLengthV,       // out:
  double & rdEstimatedAreaBound    // out:
)
{
    NL_REAL   A, lu, lv;
    NL_SURFACE * surP = GetOrCreateGwNurbPointer() ;
    NER(surP);

    NL_SER(N_SrfGetAverageLen(surP,&A,&lu,&lv));

    rdAverageLengthU = lu;
    rdAverageLengthV = lv;
    rdEstimatedAreaBound = A;

    return SM_SUCCESS;

} // end SmBSplineSurface::GetMeasures

/*******************************************************************//**
PURPOSE: Insert one new  knot into a NURBS surface either in u- or in
    v-direction. 

NOTES: The new knot must be an interior knot and the sum of
    the multiplicities of the old and the new knots must be less than
    or equal to the respective degree. 
    
      This method is only available for users with NLib
    
***********************************************************************/
SmStatus SmBSplineSurface::InsertOneKnot
(
  double dNewKnot,            // in :
  ULONG lNumKnotInsertions,   // in :
  SmSurfParamType eSurfParam  // in :
)
{
  NL_STACKS nlStacks;
  SmNLibStackHandler sSH(&nlStacks);
  NL_SURFACE sur;
  N_SrfInitArrays(&sur);

  NL_FLAG dir = NL_UDIR;
    if (eSurfParam == SM_SP_V) {
      dir = NL_VDIR;
    }

  NL_SURFACE * surP = GetOrCreateGwNurbPointer() ;
  NER(surP);

  NL_INDEX  nt = lNumKnotInsertions;
  NL_SER(N_SrfInsertKnot(surP,dNewKnot,nt,dir,&sur,&nlStacks,&nlStacks));

  const SmContext * pContext = GetContext();
  NER(pContext);
  SmBSplineSurface *pTemp = new (*pContext) SmBSplineSurface((gw_SURFACE *)&sur) ;
  SmObjDelete sClean(pTemp);

  gw_SURFACE * pNewSur = pTemp->GetOrCreateGwNurbPointer();
  pTemp->m_pNurb       = surP ;
  m_pNurb              = pNewSur;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
    if (bDebugMe) {
      sm_GraphicsLoop();
      smgfx_SetColor(0,0,1);
      DrawUV(1,1);
      Dump();
    }
#endif

  return SM_SUCCESS;

} // end SmBSplineSurface::InsertOneKnot


/*******************************************************************//**
PURPOSE: Add a set of knots into a curve.  

NOTES: Note that the knot vector
   should not contain duplicates of existing knots unless you want to
   increase the multiplicity of the knots.  This means that you should not
   put the first and last knot values of the original curve into the array.
***********************************************************************/
SmStatus SmBSplineSurface::InsertKnots
(
  SmSurfParamType    eSurfParam,  // in : one of SM_SP_U or SM_SP_V, knot vector to receive more knots
  SmTArray<double> & rNewKnots    // in : array of knot values to add
)
{
  // locals
  NL_KNOTVECTOR         knt;
  NL_STACKS             nlStacks;
  SmNLibStackHandler sSH(&nlStacks);
  NL_SURFACE          * surP = GetOrCreateGwNurbPointer() ; NER(surP);
  NL_SURFACE            sur;
  N_SrfInitArrays(&sur);

  // flag for which knot vector to edit
  NL_FLAG dir = (eSurfParam == SM_SP_U) ? NL_UDIR : NL_VDIR;

  // build knot vector for new knots
  knt.m = (NL_INDEX)rNewKnots.GetSize() - 1;
  knt.U = (NL_REAL*)rNewKnots.GetDataArray();

  // Make sur with the union of surP target knot vector and rNewKNots 
  NL_SER(N_SrfInsertKnots(surP,&knt,dir,&sur,&nlStacks,&nlStacks));

  // build SmBSplineSurface from NLib sur structure
  const SmContext  * pContext = GetContext() ; NER(pContext);
  SmBSplineSurface * pTemp    = new (*pContext) SmBSplineSurface((gw_SURFACE *)&sur) ;
  SmObjDelete sClean(pTemp);

  // swap the new NewBSplineSurface->m_pNurb with this->m_pNurb (NewBSplineSurface is temp and is deleted when exiting scope) 
  gw_SURFACE * pNewSur = pTemp->GetOrCreateGwNurbPointer();
  pTemp->m_pNurb       = surP ;
  m_pNurb              = pNewSur;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  if (bDebugMe) 
    { Dump();

      sm_GraphicsLoop();
      smgfx_SetLook(1,2, 0,0,1); DrawUV(1,1); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // all done
  return SM_SUCCESS;

} // end SmBSplineSurface::InsertKnots

/*******************************************************************//**
PURPOSE: Insert knots into this curve such that the maximum distance
    between the points on the curve is dDistance.

NOTES: 
***********************************************************************/
SmStatus SmBSplineSurface::InsertKnotsByDistance
(
  SmSurfParamType eSurfParam,     // in :
  double dDistance                // in :
)  
{
    double dData[256];
    SmTArray<double> sKnots(256,dData);
    double dData2[256];
    SmTArray<double> sNewKnots(256,dData2);
    SmExtent2d sDomain = GetNaturalUVDomain();
    GetKnots(eSurfParam,sKnots);
    for (ULONG j=1; j<sKnots.GetSize(); j++) {
        SmExtent1d sIvl(sKnots[j-1],sKnots[j]);
        if (sIvl.GetLength() < SM_EFF_ZERO_SQRT) continue;
        double dKnotDist;
        SER(CalculateApproxIsoCurveDistance(eSurfParam,sDomain,sIvl,5,4,dKnotDist));
        ULONG lNumSubdivides = (ULONG)(dKnotDist / dDistance);
        for (ULONG k=0; k<lNumSubdivides; k++) {
            double dT = (k+1.0) / (lNumSubdivides+1.0);
            sNewKnots.Add(sIvl.Evaluate(dT));
        }
    }
    SER(InsertKnots(eSurfParam,sNewKnots));
    return SM_SUCCESS;

} // end SmBSplineSurface::InsertKnotsByDistance


/*******************************************************************//**
PURPOSE: This routine computes a NURBS surface interpolation to a given
    set of (nxm)points. 

NOTES: Optionally, users may input boundary curves and Parameterizations.
    
      If boundary curves (cpOptBdryCurves) were given, the first
    two curves in the array should be u-boundaries & the last two should be
    v-boundaries. This method is only available for users with NLib
    
***********************************************************************/
SmStatus SmBSplineSurface::InterpolatePoints
(
  const SmContext & crContext,                        // in :
  const SmTArray<SmPoint3d> & crPoints,               // in : Contains lNumRows x lNumCols points
  ULONG lNumRows,                                     // in :
  ULONG lNumCols,                                     // in :
  ULONG lUDegree,                                     // in :
  ULONG lVDegree,                                     // in :
  const SmTArray<SmBSplineCurve*> * cpOptBdryCurves,  // in : The first two will be U-boundaries and the last two will be V-boundaries
  const SmTArray<double> * cpOptUParams,              // in :
  const SmTArray<double> * cpOptVParams,              // in :
  SmInterpolationType eParameterization,              // in : SM_IT_UNIFORM:
                                                      //      SM_IT_CENTRIPETAL:
                                                      //      SM_IT_CHORDLENGTH:
  SmBSplineSurface *& rpNewSurf                       // out:
)
{
    SM_ASSERT(lUDegree < lNumRows && lVDegree < lNumCols);

    NL_STACKS nlStacks;
    SmNLibStackHandler sSH(&nlStacks);
    NL_SURFACE sur;
    N_SrfInitArrays(&sur);

    NL_INDEX  r = lNumRows-1;
    NL_INDEX  s = lNumCols-1;
    NL_POINT   **P = N_AllocPt2dArray(r,s,&nlStacks);

    ULONG lIndex =0;
    for (ULONG i=0; i<lNumRows; i++) {
        for (ULONG jj=0; jj<lNumCols; jj++) {
            SmPoint3d sPnt = crPoints[lIndex++];
            COPY_XYZ(sPnt,P[i][jj]);
        }
    }

    NL_DEGREE p = (NL_DEGREE)lUDegree;
    NL_DEGREE q = (NL_DEGREE)lVDegree;
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
    NL_REAL  *u = NULL, *v = NULL;
    if (cpOptBdryCurves || (cpOptUParams && cpOptVParams)) {
        u = N_AllocReal1dArray(r,&nlStacks);
        v = N_AllocReal1dArray(s,&nlStacks);
        if (cpOptUParams && cpOptVParams) {
            // Parameterizations are given
            SM_ASSERT(cpOptUParams->GetSize() == lNumRows);
            for (ULONG iii=0; iii<lNumRows; iii++) {
                u[iii] = (*cpOptUParams)[iii];
            }
            SM_ASSERT(cpOptVParams->GetSize() == lNumCols);
            for (ULONG jjj=0; jjj<lNumCols; jjj++) {
                v[jjj] = (*cpOptVParams)[jjj];
            }
        }
        else {
            // Derive Parameterizations
            NL_SER(N_FitCalcSrfParamValues((void **)P,r,s,NL_EPOINT,par,u,v));
        }
    }

    if (cpOptBdryCurves) {
        // Boundary curves are given
        if (cpOptBdryCurves->GetSize() != 4) SER(SM_ERR);
        NL_CURVE  *bndU[2];
        NL_CURVE  *bndV[2];
        // The first two will be U-boundaries and
        // the last two will be V-boundaries
        for (ULONG ii=0; ii<4; ii++) {
            SmBSplineCurve * pCurve = (*cpOptBdryCurves)[ii];
            if (pCurve->IsRational()) {
                SER(SM_ERR); // Only non-rational is allowed 
            }
            if (ii < 2) {
                bndU[ii] = sm_CreateNlibCurve(pCurve,nlStacks);
            }
            else {
                bndV[ii-2] = sm_CreateNlibCurve(pCurve,nlStacks);
            }
        }
        // Make curves compatible
        NL_SER(N_CrvsMakeCompatible(bndU,1,&nlStacks)); 
        NL_SER(N_CrvsMakeCompatible(bndV,1,&nlStacks));
        // Surface interpolation to points and boundaries
        NL_SER(N_FitSrfInterpBoundary(P,r,s,bndU,bndV,u,v,&sur,&nlStacks));
    }
    else if (cpOptUParams && cpOptVParams) {
        // Interpolation with given Parameterizations
        NL_SER(N_FitSrfInterpTangents(P,r,s,u,v,NULL,NULL,p,q,NULL,NULL,NL_TANGENT,&sur,&nlStacks));
    }
    else if (p ==3 && q == 3) {
        // Create C1-continuous nonrational bicubic surface
        // Input parametrization is ignored
        NL_SER(N_FitSrfInterpBicubic(P,r,s, NL_AKIMA,&sur,&nlStacks));
    }
    else {
        // Global surface interpolation
        NL_SER(N_FitSrfToPts(P,r,s,p,q,par,&sur,&nlStacks));
    }

    rpNewSurf = new (crContext) SmBSplineSurface((gw_SURFACE *)&sur) ; // gwc: was TRUE = run AssertValidAndHeal on New BSplineSurface
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
    if (bDebugMe) {
        sm_GraphicsLoop();
        smgfx_SetColor(0,0,1);
        rpNewSurf->DrawUV(1,1);
        rpNewSurf->Dump();
    }
#endif

    return SM_SUCCESS;

} // end SmBSplineSurface::InterpolatePoints

/*******************************************************************//**
PURPOSE: Compute the offset of a curve on a surface, at a +- offset
     distance along the surface

NOTES:      The surface is extended to allow the offset, or 
     for the input curve to stray outside the domain of the input surface.
     The output curve is not trimmed to the boundary of the input surface,
     and may also contain self-intersections.

***********************************************************************/

SmStatus SmBSplineSurface::OffsetCurveAlongSurface
(
  const SmContext &crContext,   // in :
  SmBSplineCurve *pCrv,         // in : curve must lie on or near surface
  double dOffset,               // in : + or - for which side of crv
  double dTol,                  // in : tolerance used for intersection ( use 0,001 )
  SmBSplineCurve *& pOffsetCrv  // out: offset curve
)
{

    // Resize the surface to extend by double the offset distance
    SmSurface        *pNewSurf = NULL;
    SmBSplineSurface *pSrf = NULL ;
    double dSize = 2.0 *((dOffset > 0.0)? dOffset : - dOffset); 
        
    this->CreateExtendedSurface(crContext, dSize, SM_CT_G1R, pNewSurf);
    SmBSplineSurface *pNewSurface = SM_CAST_PTR(SmBSplineSurface, pNewSurf);
    SmObjDelete sCleanSurf(pNewSurface), sCleanSurface(pNewSurf);
    pSrf = pNewSurface;
 
    SmExtent1d CrvDomain = pCrv->GetNaturalInterval();
    SmExtent2d sUVDomain = pSrf->GetNaturalUVDomain(); 
    
    double t; 
    SmPoint3d sFndPnt;
    double dCarcParam = 0.5; // default solution is midpoint of arc
    
    SmTArray <double> pParameters;
    SmTArray <SmPoint3d> pPoints;
    SmTArray <SmPoint3d> OffsetPoints;
    
    // NLib stuff
    NL_STACKS S;        
    NL_CURVE Circle;
    
    N_InitNurbs(&S);
    N_CrvInitArrays(&Circle); 
    // End Nlib Stuff

    // Tessellate curve to get sample points for offsetting
    pCrv->Tessellate(CrvDomain, 0.2, 5.0, 6, &pParameters, NULL);
    
    for (ULONG ii = 0; ii < pParameters.GetSize(); ii++)
    {
        // t is parameter along pCrv
        t = pParameters[ii];                     
        
        SmPoint3d CrvDer[2];
        SER(pCrv->Evaluate(t, 1, TRUE, CrvDer));
        
        double Dist = 0.0;
        // Note if the original curve is not ON the surface we will miss some
        // points here.....bad results if CrvTol too small
        double dCrvTol = 0.01;
        SmSolutionArray sSolutions;
        SER(pSrf->GlobalPointSolve(sUVDomain,
            SM_SO_MINIMIZE, CrvDer[0],
            dCrvTol, &Dist, SM_SR_SINGLE,
            sSolutions));
        if (sSolutions.GetSize() < 1)
        {
            pParameters.RemoveAt(ii, 1);
            ii--;
            continue;// no point on surface 
        }
        
        SmPoint2d UV = SmPoint2d(sSolutions[0].m_vStart[0], sSolutions[0].m_vStart[1]);
        SmVector3d SrfNorm;
        pSrf->EvaluateNormal(UV, TRUE, TRUE, SrfNorm);
        SrfNorm.Unitize();
        CrvDer[1].Unitize();
        SmVector3d yDir = SrfNorm * CrvDer[1]; 
        
        
        // Enter the world of NLib
        NL_POINT  YDir;
        YDir.x = yDir.x;
        YDir.y = yDir.y;
        YDir.z = yDir.z;
        NL_POINT  Center;
        Center.x = CrvDer[0].x;
        Center.y = CrvDer[0].y;
        Center.z = CrvDer[0].z;
        NL_POINT  Norm;
        Norm.x = SrfNorm.x;
        Norm.y = SrfNorm.y;
        Norm.z = SrfNorm.z;
        
        // re-use previously allocated circle memory
        if (dOffset < 0.0)
            N_CreateQuadraticArc(Center, Norm, YDir, -dOffset, 180.0, 360.0, &Circle, &S);
        else
            N_CreateQuadraticArc(Center, Norm, YDir,  dOffset,  0.0, 180.0, &Circle, &S);;
        
        SmBSplineCurve *pCircle = new(crContext) SmBSplineCurve(3, (gw_CURVE *)&Circle);
        SmObjDelete sClean(pCircle);
        // leave  the world of NLib
        
        SmSolutionArray sSols;
        SmStatus status = pSrf->GlobalCurveIntersect(sUVDomain,
            *pCircle, pCircle->GetNaturalInterval(),
            dTol, sSols);
        
        if (status != SM_SUCCESS) 
            continue;
               
        if (sSols.GetSize() < 1) // if we fail to get a solution, evaluate the 
                                 // circle at the same t as the previous circle solution
        {
            SER(pCircle->EvaluatePoint(dCarcParam, sFndPnt));
            OffsetPoints.Add(sFndPnt);
            continue;
        }
        SmSolution rSolution = sSols[0]; 
        if (sSols.GetSize() > 1)
        {
            // if there are 2+solution, find the one with the closest 
            // 't' to the previous dCarcParam              
            ULONG  kkk = 0;
            double dist = 9999.;
            for (ULONG kk = 0; kk < sSols.GetSize(); kk++)
            {
                double dT = sSols[kk].m_vStart[0];   // Note that curve is first parameter
                if (fabs( dT - dCarcParam) < dist)
                {
                    dist = fabs( dT - dCarcParam);
                    kkk = kk;
                }
            }
            rSolution = sSols[kkk]; // the best solution
        }
        
        // with one solution, or the best of many, evaluate u,v
        SmPoint2d sUVFound(rSolution.m_vStart[1], rSolution.m_vStart[2]);
        SER(pSrf->EvaluatePoint(sUVFound, sFndPnt)); 
        
        // get circle parameter to save for next 't' default
        dCarcParam = rSolution.m_vStart[0];
                       
        OffsetPoints.Add(sFndPnt);
    } // for next t
    
    // Build Curve from interpolating points    
    SER(SmBSplineCurve::InterpolatePoints(crContext,
              OffsetPoints, &pParameters, 3, NULL, NULL, FALSE, 
              SM_IT_CHORDLENGTH, pOffsetCrv));
    
    N_EndNurbs(&S);
    
    return SM_SUCCESS;

} // end SmBSplineSurface::OffsetCurveAlongSurface

/*******************************************************************//**
PURPOSE: Refine a NURBS surface with a given knot vector in either
    u- or  v-direction. 

NOTES: It is  assumed that the new knot vector fits
    into the old  ones.
    
      This method is only available for users with NLib
    
***********************************************************************/
SmStatus SmBSplineSurface::RefineSurface
(
  const SmTArray<double> & crNewKnots,    // in :
  SmSurfParamType eSurfParam              // in :
)
{
    NL_STACKS nlStacks;
    SmNLibStackHandler sSH(&nlStacks);
    NL_SURFACE sur;
    N_SrfInitArrays(&sur);

    NL_KNOTVECTOR  *knt;
    NL_INDEX       m = crNewKnots.GetSize()-1;
    knt = N_AllocKnotVectorAndArray(m,&nlStacks);
    NL_REAL *U = knt->U;
    for (ULONG i=0; i<crNewKnots.GetSize(); i++) {
        U[i] = crNewKnots[i];
    }

    NL_FLAG dir = NL_UDIR;
    if (eSurfParam == SM_SP_V) {
        dir = NL_VDIR;
    }

    NL_SURFACE * surP = GetOrCreateGwNurbPointer() ;
    NER(surP);

    NL_SER(N_SrfInsertKnots(surP,knt,dir,&sur,&nlStacks,&nlStacks));

    const SmContext * pContext = GetContext();
    NER(pContext);

    SmBSplineSurface *pTemp = new (*pContext) SmBSplineSurface((gw_SURFACE *)&sur) ;
    SmObjDelete sClean(pTemp);

    gw_SURFACE * pNewSur = pTemp->GetOrCreateGwNurbPointer();
    pTemp->m_pNurb       = surP ;
    m_pNurb              = pNewSur;
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
    if (bDebugMe) {
        sm_GraphicsLoop();
        smgfx_SetColor(0,0,1);
        DrawUV(1,1);
        Dump();
    }
#endif

    return SM_SUCCESS;

} // end SmBSplineSurface::RefineSurface

/*******************************************************************//**
PURPOSE: Remove all removable knots from a NURBS surface with optional
    constraints on some of the knots that are not to be removed.

NOTES: This method is only available for users with NLib
    
***********************************************************************/
SmStatus SmBSplineSurface::RemoveKnots
  (double dThisApproxTol3d,                   // in : max allowed Current/New Curve deviation
   SmBoolean bRemoveUKnots,                   // in : TRUE = Constrain end derivatives, FALSE = don't
   SmBoolean bRemoveVKnots,                   // in : TRUE = Constrain end derivatives, FALSE = don't
   const SmTArray<double> * cpOptKeptUKnots,  // out: u-knots that will NOT be removed
   const SmTArray<double> * cpOptKeptVKnots)  // out: v-knots that will NOT be removed
{
    NL_STACKS nlStacks;
    SmNLibStackHandler sSH(&nlStacks);
    NL_SURFACE sur;
    N_SrfInitArrays(&sur);

    NL_FLAG dir = NL_UVDIR;
    if (bRemoveUKnots && bRemoveVKnots) {
        dir = NL_UVDIR;
    }
    else if (bRemoveUKnots) {
        dir = NL_UDIR;
    }
    else if (bRemoveVKnots) {
        dir = NL_VDIR;
    }

    NL_SURFACE * surP = GetOrCreateGwNurbPointer() ;
    NER(surP);

    if (cpOptKeptUKnots || cpOptKeptVKnots) {
        NL_PARAMETER  *UK = NULL, *VK = NULL;
        NL_INDEX      mu = 0, mv = 0;
        if (cpOptKeptUKnots) {
            ULONG lCount = cpOptKeptUKnots->GetSize();
            mu = lCount-1;
            UK = N_AllocReal1dArray(mu,&nlStacks);
            for (ULONG iii=0; iii<lCount; iii++) {
                UK[iii] = (*cpOptKeptUKnots)[iii];
            }
        }
        if (cpOptKeptVKnots) {
            ULONG lCount = cpOptKeptVKnots->GetSize();
            mv = lCount-1;
            VK = N_AllocReal1dArray(mv,&nlStacks);
            for (ULONG jjj=0; jjj<lCount; jjj++) {
                VK[jjj] = (*cpOptKeptVKnots)[jjj];
            }
        }
        NL_SER(N_SrfRemoveKnotsConstraints(surP,UK,mu,VK,mv,dThisApproxTol3d,dir,&sur,&nlStacks));
    }
    else {
        NL_SER(N_SrfRemoveAllKnots(surP,dThisApproxTol3d,dir,&sur,&nlStacks));
    }

    const SmContext * pContext = GetContext();
    NER(pContext);
    SmBSplineSurface *pTemp = new (*pContext) SmBSplineSurface((gw_SURFACE *)&sur) ;
    SmObjDelete sClean(pTemp);

    gw_SURFACE * pNewSur = pTemp->GetOrCreateGwNurbPointer();
    pTemp->m_pNurb       = surP ;
    m_pNurb              = pNewSur;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
    if (bDebugMe) {
        sm_GraphicsLoop();
        smgfx_SetColor(0,0,1);
        DrawUV(1,1);
        Dump();
    }
#endif

    return SM_SUCCESS;

} // end SmBSplineSurface::RemoveKnots

/*******************************************************************//**
PURPOSE: Remove one knot multiple times from a NURBS surface either               
         in u- or in v-direction.                                                 
                                                                                  
NOTES:  The knot must be an interior knot.                                        
        Knots not removed when removal changes the shape of the surface           
        by more than dThisApproxTol3d.
***********************************************************************/
SmStatus SmBSplineSurface::RemoveOneKnot
 (double          dKnot,             // in : Knot value to be removed
  ULONG           lNumKnotsRemoval,  // in : number of times to remove the knot
  SmSurfParamType eSurfParam,        // in : SM_SP_U = Remove in u-direction
                                     //      SM_SP_V = Remove in v-direction
  double          dThisApproxTol3d,  // in : max allowed Current/New Curve deviation
  ULONG         & rlNumKnotsRemoved) // out: Number of knots removed
{
  // locals
  const SmContext  * pContext = GetContext();                NER(pContext) ;
  NL_SURFACE          * surP     = GetOrCreateGwNurbPointer() ; NER(surP) ;
  NL_SURFACE            sur ;
  NL_FLAG               dir = (eSurfParam == SM_SP_V) ? NL_VDIR : NL_UDIR ;
  NL_INDEX              nt = lNumKnotsRemoval ;
  NL_INDEX              rt ;
  NL_STACKS             nlStacks ;
  SmNLibStackHandler sSH(&nlStacks) ;
  N_SrfInitArrays(&sur) ;

  // pass the call along
  NL_SER(N_SrfRemoveKnotConditional(surP,               // in : Target NURBS surface
                                    dKnot,              // in : Knot value to be removed
                                    nt,                 // in : number of times to remove the knot
                                    dThisApproxTol3d,   // in : Tolerance to check removability
                                    dir,                // in : NL_UDIR: Remove in u-direction
                                                        //      NL_VDIR: Remove in v-direction
                                    &rt,                // out: Number of knots removed
                                    &sur,               // out: Surface copy with knot removed
                                    &nlStacks)) ;       // in : surQ's stack
  rlNumKnotsRemoved = rt ;

  // when a knot was removed
  if(rt > 0 )
    {
      // Build SMLib SPlineSurface from NLib gw_SURFACE
      SmBSplineSurface *pTemp = new (*pContext) SmBSplineSurface((gw_SURFACE *)&sur) ;
      SmObjDelete sClean(pTemp);

      // swap the m_pNurb pointers
      gw_SURFACE * pNewSur = pTemp->GetOrCreateGwNurbPointer();
      pTemp->m_pNurb       = surP ;
      m_pNurb              = pNewSur;

    } // end knot was removed check

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  if (bDebugMe) 
    {
      Dump();

      smgfx_SetColor(0,0,1); DrawUV(1,1); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // all done
  return SM_SUCCESS;

} // end SmBSplineSurface::RemoveOneKnot

/*******************************************************************//**
PURPOSE: Make a BSplineSurface non-rational by dividing by the weight

NOTES:  
    This is a simple data conversion. It modifies the shape of the surface.
    It is not a fitting process.
    This method is only available for users with NLib
    
***********************************************************************/
SmStatus SmBSplineSurface::MakeNonRational
  ()
{

    NL_CPOINT *Cp;
    NL_SURFACE * sur = GetOrCreateGwNurbPointer() ;
    NER(sur);
    NL_INDEX n = sur->net->n;
    NL_INDEX m = sur->net->m;
    for (NL_INDEX ii = 0; ii <= n; ii++)
    {
        for (NL_INDEX jj = 0; jj <= m; jj++)
        {
            Cp = &(sur->net->Pw[ii][jj]);
            if (Cp->w !=NL_NOW)
            {
                NL_REAL w = Cp->w;
                Cp->x /= w;
                Cp->y /= w;
                if (Cp->z != NL_NOZ) Cp->z /= w;
                Cp->w = NL_NOW;
            }
        }
    }

    return SM_SUCCESS;

} // end SmBSplineSurface::MakeNonRational

/*******************************************************************//**
PURPOSE: Reparameterize the domain of a B-Spline Surface

NOTES: This method is only available for users with NLib
***********************************************************************/
SmStatus SmBSplineSurface::Reparameterize
  (const SmExtent2d & crTrimDomain        // in : new parameter range for BSPlineSurface
      // SmBoolean    bUpdateSTEPParams   // in : TRUE = Rebuild STEP params to match Bspline parameters
  )                                       //      FALSE= skip rebuild step params - needed for analytic constructors 
                                          //      Not used.
{
  Notify(SM_NO_PRE_EDIT, this, SM_NO_GET_OWNER(this), NULL);

  NL_RECTANGLE R;

  R.ul = crTrimDomain.GetMin().x;
  R.ur = crTrimDomain.GetMax().x;
  R.vb = crTrimDomain.GetMin().y;
  R.vt = crTrimDomain.GetMax().y;

  NL_SURFACE * surP = GetOrCreateGwNurbPointer();
  NER(surP);

  N_SrfReparamToInterval(surP,R,NL_UVDIR); 
  Notify(SM_NO_POST_EDIT, this, SM_NO_GET_OWNER(this), NULL);

  // give analytics a chance to update their nested STEP data
  // No, don't do this here.  This should deal with SmBSplineSurface stuff only.
  // To do something like this, derived classes can implement this method, it's virtual.
  // [B375]
  // if(bUpdateSTEPParams)
  //   { RebuildSTEPFromNURBParameters() ; }

  // all done
  SM_DUMP_AND_ASSERT2_VALID(this) ;
  return SM_SUCCESS;

} // end SmBSplineSurface::Reparameterize

/*******************************************************************//**
PURPOSE: Reparameterize the domain of a B-Spline Surface

NOTES: This method is only available for users with NLib
    
***********************************************************************/
SmStatus SmBSplineSurface::ReparametrizeWithArcLength
  ()
{
  NL_STACKS nlStacks;
  SmNLibStackHandler sSH(&nlStacks);
  NL_SURFACE sur;
  N_SrfInitArrays(&sur);
  Notify(SM_NO_PRE_EDIT, this, SM_NO_GET_OWNER(this), NULL);

  NL_SURFACE * surP = GetOrCreateGwNurbPointer() ;
  NER(surP);

  NL_REAL tol = 0.01;  //Relative tolerance 
  NL_SER(N_SrfReparamArcLength(surP,tol,&sur,&nlStacks)); 

  const SmContext * pContext = GetContext();
  NER(pContext);
  SmBSplineSurface *pTemp = new (*pContext) SmBSplineSurface((gw_SURFACE *)&sur) ;
  SmObjDelete sClean(pTemp);

  gw_SURFACE * pNewSur = pTemp->GetOrCreateGwNurbPointer();
  pTemp->m_pNurb       = surP ;
  m_pNurb              = pNewSur;
  Notify(SM_NO_POST_EDIT, this, SM_NO_GET_OWNER(this), NULL);

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  if (bDebugMe) {
      sm_GraphicsLoop();
      smgfx_SetColor(0,0,1);
      DrawUV(1,1);
      Dump();
  }
#endif
  
  // give analytics a chance to update their nested STEP data
  RebuildSTEPFromNURBParameters() ;

  // all done
  SM_DUMP_AND_ASSERT2_VALID(this) ;
  return SM_SUCCESS;

} // end SmBSplineSurface::ReparametrizeWithArcLength


/*******************************************************************//**
PURPOSE: This surface routine reverses a surface, i.e. it  will trace
    out in reverse order either in u- or in v-direction

NOTES: This method is only available for users with NLib
    
***********************************************************************/
SmStatus SmBSplineSurface::Reverse
  (SmSurfParamType eSurfParam)
{
    NL_STACKS nlStacks;
    SmNLibStackHandler sSH(&nlStacks);

    NL_FLAG dir = NL_UDIR;
    if (eSurfParam == SM_SP_V) {
        dir = NL_VDIR;
    }
    
    NL_SURFACE * sur = GetOrCreateGwNurbPointer();

    Notify(SM_NO_PRE_EDIT, this, SM_NO_GET_OWNER(this), NULL);
    NL_SER(N_SrfReverse(sur,dir,sur,&nlStacks));
    Notify(SM_NO_POST_EDIT, this, SM_NO_GET_OWNER(this), NULL);

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
    if (bDebugMe) {
        sm_GraphicsLoop();
        smgfx_SetColor(0,0,1);
        DrawUV(1,1);
        Dump();
        sm_GraphicsLoop();
SmBoolean bDebugOutput = FALSE;
        if (bDebugOutput)
            WriteToFile(_T("surf.sms"));
    }
#endif

  // give analytics a chance to update their nested STEP data
  RebuildSTEPFromNURBParameters() ;

  // all done
  SM_DUMP_AND_ASSERT2_VALID(this) ;
  return SM_SUCCESS;

} // end SmBSplineSurface::Reverse


/*******************************************************************//**
PURPOSE: Test surface for degenerate control points and return location

NOTES: This method is only available for users with NLib

METHOD --- search all control Points to see if any share a duplicate location
           within tolerance.  When one is found set Nu, Nv, Nw to the
           index of the first such occurance and return
           SM_ERR, else return SM_SUCCESS.
***********************************************************************/
SmStatus SmBSplineSurface::TestDegenerate
  (double Tolerance,  // in : min distance between unique ControlPoint positions.
   ULONG  &rNu,       // out: index of first ControlPoint with a duplicate or 0
   ULONG  &rNv)       // out: index of first ControlPoint with a duplicate or 0 
  const
{
  // locals
  NL_SURFACE * surP = ((SmBSplineSurface *)this)->GetOrCreateGwNurbPointer();
  NER(surP);
  NL_INDEX nu=0, nv=0;

  // pass the call along
  short fRtn = N_SrfHasEqualCPts( surP, Tolerance, &nu, &nv) ;

  // set output
  rNu = nu;
  rNv = nv;

  return fRtn == 0 ? SM_SUCCESS : SM_ERR ;

} // end SmBSplineSurface::TestDegenerate

/*******************************************************************//**
PURPOSE: Text Surface corners for du x dv = 0.0 (no normal)

NOTES:  Tol is min length of du x dv (not normalised)
            May be a pole, a cusp,  or du->, dv-> may be co-linear.
            This method is only available for users with NLib
            
***********************************************************************/
SmStatus SmBSplineSurface::TestCornerNormals
  (double Tol) 
 const
{   
    SmPoint2d sUV;
    SmPoint3d sPnt;
    SmVector3d S, sDU, sDV;
    double L, TolSq;
    SmExtent2d sUVDomain = GetNaturalUVDomain();

    TolSq = Tol*Tol;

    // Evaluate derivatives at all uv corners
    sUV = sUVDomain.Evaluate(0.0,0.0);
    SER(Evaluate1stDerivatives(sUV,TRUE,TRUE,sPnt,sDU,sDV));
    S = sDU * sDV;     L = S.LengthSquared();    
    if (L < TolSq) SER(SM_ERR_WARNING);

    sUV = sUVDomain.Evaluate(0.0,1.0);    
    SER(Evaluate1stDerivatives(sUV,TRUE,TRUE,sPnt,sDU,sDV));
    S = sDU * sDV;     L = S.LengthSquared();    
    if (L < TolSq) SER(SM_ERR_WARNING);

    sUV = sUVDomain.Evaluate(1.0,1.0);    
    SER(Evaluate1stDerivatives(sUV,TRUE,TRUE,sPnt,sDU,sDV));
    S = sDU * sDV;     L = S.LengthSquared();    
    if (L < TolSq) SER(SM_ERR_WARNING);
 
    sUV = sUVDomain.Evaluate(1.0,0.0);    
    SER(Evaluate1stDerivatives(sUV,TRUE,TRUE,sPnt,sDU,sDV));
    S = sDU * sDV;     L = S.LengthSquared();    
    if (L < TolSq) SER(SM_ERR_WARNING);

    return SM_SUCCESS;

} // end SmBSplineSurface::TestCornerNormals

/*******************************************************************//**
PURPOSE: Local routine, helper for TrimWithDomain(): Check proposed trim boundaries
   against the original input domain: they must not make that domain smaller.

NOTES:
   If adjusted, dTMin and/or dTMax will be set to the first knot value
   found that is not interior to the given domain.
***********************************************************************/
static void sm_CheckTrimBoundaries
 (const SmTArray<double> & rKnots,   // in : knot array to check
  double                 & dTMin,    // i/o: proposed min trim boundary, can't be inside the domain.
  int                      lMinIdx,  // in : index into rKnots of dTMin, negative if not set.
  double                 & dTMax,    // i/o: proposed max trim boundary
  int                      lMaxIdx,  // in : index into rKnots of dTMax, negative if not set.
  const SmExtent2d       & rDomain,  // in : dTMin and dTMax must not be interior to this domain.
  SmSurfParamType          eUVDir)   // in : 
{
  // locals
  int   ii ;
  ULONG li ;

  // pick knot direction
  SmExtent1d sDom =   (eUVDir == SM_SP_U) 
                    ? rDomain.GetUInterval() 
                    : rDomain.GetVInterval() ;

  // When min knot index and TMin are in the interior of sDom
  if(   lMinIdx >= 0 
     && dTMin   > sDom.GetMin() )
    {
      // iter every knot looking for 1st knot not within current sDom interval 
      for(ii=lMinIdx;ii>=0;ii--) 
        {
          // found 1st knot outside of sDom interval
          if(rKnots[ii] <= sDom.GetMin())
            {
              dTMin = rKnots[ii];
              break;
            }
        }

      // before the beginning
      if ( ii < 0 )
        { dTMin = rKnots[ 0 ]; } 
    } // end minKnotIndx and Tmin are inside of sDom check

  // when max knot index and tMax are in the interior of sDom
  if(   lMaxIdx >= 0 
     && dTMax < sDom.GetMax() )
    {
      // iter every knot looking for 1st knot not within curren sDom interval
      for(li=lMaxIdx;li<rKnots.GetSize();li++)
        {
          // find 1st knot outside of sDom interval
          if(rKnots[li] >= sDom.GetMax())
            {
              dTMax = rKnots[li];
              break;
            }
        }

      // past the end
      if ( li >= rKnots.GetSize() )
        { dTMax = rKnots[ rKnots.GetSize()-1 ]; }  
    } // end maxKnotIndx and Tmax are inside of sDom check

  // all done
  return;

} // end local sm_CheckTrimBoundaries()

/*******************************************************************//**
PURPOSE: Trim a B-Spline surface to correspond to a subset
    of the original surface trimmed to a square UV boundary.  
    
    The surface's natural UV Domain is modified to equal the input
    trim boundary domain.

NOTES:
  This function preserves the input surface geometry and its
  parameterization at the domain corners exactly, however the surface's 
  parameterization between domain corners may vary slightly but by
  amounts easily larger than reasonable tolerance sizes.  

  As such, existing Edgeuse->UVTrimCurves that reference the surface
  being trimmed should be deleted and rebuilt after this call.

  1. The surface's natural UV Domain is modified to be near the input
     trim boundary domain.
  2. The trimBoundary may be snapped to existing knot values.
  3. The input trimBoundary domain is set to equal the surface's
     new natural domain on exit.

METHOD
  1. Trim boundaries close to existing knots are snapped to 
     (or away from) existing knot values to prevent the creation of 
     short (as measured in parameter space) spans.
  1a. The input rTrimDomain is modified if any trim boundary is snapped.

  2. A new surface is created from the old surface whose natural domain
     is equal to the requested trim boundary by inserting knots
     at the trim values.  Knots and Control Points outside
     of the requested trim boundary are removed from the new surface.
  
  3. The SmBSPlineSurface->m_pNurb pointers of this SmBSplineSurface
     and the new one are swapped making the trimmed surface the
     official definition. 
  
  4. The new surface with the old m_pNurb pointer is deleted when 
     exiting the function's scope.  
   
  The definition of the NURB surface pointer under this SMBSplineSurface
  is modified.
***********************************************************************/
SmStatus SmBSplineSurface::TrimWithDomain
  (SmExtent2d & rTrimDomain)       // i/o: requested surface new natural domain
                                   //      set to modified surface's natural domain.
                                   //      Input trim values may be snapped
                                   //      to (or away) from existing knot values
                                   //      to prevent the creation of short spans.
{
  // get current natural boundary
  SmExtent2d sNaturalDomain = GetNaturalUVDomain();

  // check input - trim domain not contained within natural domain (no tolerance allowed)
  if (!rTrimDomain.IsContainedBy(sNaturalDomain, SM_EFF_ZERO_PARAM)) 
    { SER(SM_ERR); }

  // no work - natural domain is same size as trim domain
  if (sNaturalDomain.IsContainedBy(rTrimDomain, SM_EFF_ZERO_PARAM)) 
    { return SM_SUCCESS; }
  
  NL_STACKS     nlStacks;
  SmNLibStackHandler sSH(&nlStacks);

  // init NLIb style surface (no internal ControlPoint, Weight, or knot memory)
  NL_SURFACE sur1;
  N_SrfInitArrays(&sur1);

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;

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
      smgfx_SetLook( 1, 2, 0, 0, 1 ); if(pBrep) { pBrep->Draw( TRUE ); sm_GraphicsLoop(); }
      smgfx_SetLook(1,2, 0,1,1) ; this->DrawUV(8,8,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook( 1, 2, 0, 0, 0 ); if(pFace) { pFace->Draw( SM_DM_CROSSHATCH ); sm_GraphicsLoop(); }
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // get trim domain corner points
  NL_REAL ul = rTrimDomain.GetMin().x;
  NL_REAL ur = rTrimDomain.GetMax().x;
  NL_REAL vl = rTrimDomain.GetMin().y;
  NL_REAL vr = rTrimDomain.GetMax().y;

  // Never allow these adjustments to make the trim domain smaller.
  // That can result in a surface with a smaller domain than a face,
  // which really causes problems.  This can happen, for instance,
  // when the knot spacing is much smaller than the tolerances we use.
  // (We could probably adjust our tolerances, but that gets dicey,
  // it's safer just to correct it when it happens.)  [B306]
  int idxMin, idxMax;

  // don't build too small spans in u direction. 
  // snap U trim boundaries to existing U knots or
  //   away from U knots to prevent making a span less than 10*tolerance
  {
    // scale tolerances to domain size
    double dTolMin = SM_EFF_ZERO_SQRT*(1.0+smos_Fabs(ul));
    double dTolMax = SM_EFF_ZERO_SQRT*(1.0+smos_Fabs(ur));

    idxMin = idxMax = -1;
    
    // get current U knots
    SmTArray<double> sKnots;
    GetKnots(SM_SP_U,sKnots);

    // for every U knot - looking to snap trim boundaries
    for (ULONG i=0; i<sKnots.GetSize(); i++) 
      {
        double dKnot = sKnots[i];

        // when lower trim boundary is close to knot
        if (smos_Fabs(dKnot - ul) < dTolMin) 
          {
            // when trim boundary is slightly larger than knot - set trim boundary to knot
            if (ul >= dKnot || i == 0) 
              {
                ul = dKnot;
                idxMin = i;
              }
            else // space trim boundary more than tol away from knot
              { // Set UL to either the previous knot or slightly less than current knot
                ul = dKnot - 10.0 * dTolMin;
                idxMin = i;
                if (ul < sKnots[i-1]) 
                  {
                    ul = sKnots[i-1];
                    idxMin = i-1;
                  }
              }
          } // end this knot close to lower trim boundary check

        // when upper trim boundary is close to knot
        if (smos_Fabs(dKnot - ur) < dTolMax) 
          {
            if (ur <= dKnot || i==sKnots.GetSize()-1) 
              {
                ur = dKnot;
                idxMax = i;
              }
            else 
              {
                ur = dKnot + 10.0 * dTolMax;
                if (ur > sKnots[i+1]) 
                  {
                    ur = sKnots[i+1];
                    idxMax = i+1;
                  }
              }
          }  // end this knot close to upper trim boundary check
      } // end iter every U knot
    
    // Make sure we're not shrinking the input domain. [B306]
    // Possibly adjust ur and ul to a knot value that's not interior to rTrimDomain.
    // (Put it into a local routine to keep the code clearer.)
    sm_CheckTrimBoundaries( sKnots, ul, idxMin, ur, idxMax, rTrimDomain, SM_SP_U );

    // no work - snapped trim boundaries are within tolerance of one another
    if (smos_Fabs(ur - ul) < dTolMin + dTolMax) 
      {
        // Error here because trim points are too close together
        return SM_ERR_INVALID_INPUT;
      }

  } // end filtering new small U span widths

  // don't build too small spans in V direction. 
  // snap V trim boundaries to existing V knots or
  //   away from V knots to prevent making a span less than 10*tolerance
  {
    double dTolMin = SM_EFF_ZERO_SQRT*(1.0+smos_Fabs(vl));
    double dTolMax = SM_EFF_ZERO_SQRT*(1.0+smos_Fabs(vr));

    idxMin = idxMax = -1;
    
    // for every V knot - look to snap V trim boundaries
    SmTArray<double> sKnots;
    GetKnots(SM_SP_V,sKnots);
    for (ULONG i=0; i<sKnots.GetSize(); i++) 
      {
        double dKnot = sKnots[i];

        // snap lower V Trim boundaries as appropriate
        if (smos_Fabs(dKnot - vl) < dTolMin) 
          {
            if (vl >= dKnot || i==0)
              {
                vl = dKnot;
                idxMin = i;
              }
            else 
              {
                vl = dKnot - 10.0 * dTolMin;
                idxMin = i;
                if (vl < sKnots[i-1]) 
                  {  vl = sKnots[i-1];
                  }
              }
          } // end lower V Trim boundary near knot value check

        // snap upper V Trim Boundaries as appropriate
        if (smos_Fabs(dKnot - vr) < dTolMax) 
          {
            if (vr <= dKnot || i==sKnots.GetSize()-1) 
              {
                vr = dKnot;
                idxMin = i;
              }
            else 
              {
                vr = dKnot + 10.0 * dTolMax;
                idxMin = i;
                if (vr > sKnots[i+1]) 
                  {
                    vr = sKnots[i+1];
                    idxMin = i+1;
                  }
              }
          } // end upper V Trim boundary near knot value check
      } // end iter every V knot - looking to snap trim boundaries
    
    // Make sure we're not shrinking the input domain. [B306]
    // Possibly adjust vr and vl to a knot value that's not interior to rTrimDomain.
    sm_CheckTrimBoundaries( sKnots, vl, idxMin, vr, idxMax, rTrimDomain, SM_SP_V );
    
    // no work - snapped trim boundaries are within tolerance of one another
    //if (smos_Fabs(vr - vl) < dTolMin + dTolMax) 
    //  {
    //    // Error here because trim points are too close together
    //    return SM_ERR_INVALID_INPUT;
    //  }
  }

  // place snapped Trim Boundary values into input TrimDomain
  rTrimDomain.SetMinMax(SmPoint2d(ul,vl),SmPoint2d(ur,vr));

  // local surface pointer
  NL_SURFACE * surP = GetOrCreateGwNurbPointer();
  NER(surP);

  // extract patch defined by trim boundaries from surP into sur1
  // sur1 is a new surface. 
  NL_SER(N_SrfExtractPatch(surP,ul,ur,vl,vr,&sur1,&nlStacks,&nlStacks)); 

  // build SMLib surface from NLib sur1
  SmBSplineSurface *pTemp = new (*GetContext()) SmBSplineSurface((gw_SURFACE *)&sur1) ;

  // delete pTemp surface when exiting scope
  SmObjDelete sClean(pTemp);

  // swap the old and new BSPline->m_pNurb surface pointers. 
  Notify(SM_NO_PRE_EDIT, this, SM_NO_GET_OWNER(this), NULL);
  gw_SURFACE * pNewSur = pTemp->GetOrCreateGwNurbPointer();
  pTemp->m_pNurb       = surP ;
  m_pNurb              = pNewSur;
  Notify(SM_NO_POST_EDIT, this, SM_NO_GET_OWNER(this), NULL);

#ifdef SM_DEBUG_CODE
  if(bDebugMe)
    {
      // old idea - check max distance between surfaces (doesn't find the max gap - just useless max distance)
      SmSolutionArray sSolutions ;
      pCopySurface->GlobalSurfaceSolve(this->GetNaturalUVDomain(), 
                                       *this, 
                                       this->GetNaturalUVDomain(), 
                                       SM_SO_MAXIMIZE, SM_EFF_ZERO, NULL, NULL, 
                                       SM_SR_ALL, sSolutions) ;

      SM_DUMP_AND_ASSERT_VALID(this) ;
      sSolutions.Dump() ;

      SmFace *pFace = (SmFace *)this->GetFace() ;
      SmBrep *pBrep =   pFace ? pFace->GetBrep() : NULL ;
      SmExtent2d sDom = this->GetNaturalUVDomain();
      SmZoneTol3d sZoneTol3d =   pFace ? (double)pFace->GetTolerance() 
                    : pBrep ? (double)pBrep->GetTolerance() 
                    : 0.00001 ;

      SmSrfSrfGapFunction sSrfSrfGap( (SmXSectTol3d)sZoneTol3d,
                                      this, sDom,
                                      pCopySurface, 40, 40) ;
      sSrfSrfGap.Dump() ;


      smgfx_Erase() ;
      smgfx_SetLook( 1, 2, 0, 0, 1 ); if(pBrep) { pBrep->Draw( TRUE ); sm_GraphicsLoop(); }
      smgfx_SetLook(1,2, 0,1,1) ; this->DrawUV(8,8,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook( 1, 2, 0, 0, 0 ); if(pFace) { pFace->Draw( SM_DM_CROSSHATCH ); sm_GraphicsLoop(); }
      smgfx_SetLook(1,2, 1,0,0) ; sSrfSrfGap.Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(2,3, 1,0,0) ; sSolutions.Draw() ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // all done
  return SM_SUCCESS;

} // end SmBSplineSurface::TrimWithDomain
 
/*******************************************************************//**
PURPOSE: Output polygons for a surface to the given call back function.

NOTES: Note that right now it only outputs triangles.
            USES NLib trimmed surface tessellator
***********************************************************************/
SmStatus SmBSplineSurface::OutputPolygons
  (double                    dSurfaceChordHeightTolerance,   // in : max distance between polygon and surface
   double                    dCurveChordHeightTolerance,     // in : mas distance between polygon edge and surface
   SmBoolean                 bReverseNormals,                // in : TRUE = Reverse polygon normals prior to display
                                                             //      FALSE= don't
   SmPolygonOutputCallback & rPolygonOutput,                 // in : Chooses how and where to output the polygons
   SmGfxArraySet           * pOptGfxSet)                     // i/o: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.
                                                             //      NULL to ignore. default:[NULL]
   const
{
    const SmBSplineSurface* pSrf = this;
    const SmContext* pContext = pSrf->GetContext();

    NL_CURVE*** cuo;    // Outer trimming loops (You MUST have some)
    NL_CURVE**** cui;   // Inner trimming loops
    NL_SURFACE* sur;    // NURBS surface (knot vectors are rescaled)     
    NL_INDEX m = 0;     // Highest index of outer trimming loops     
    NL_INDEX ho[1];     // Array of highest indexes of outer loop segments     
    NL_INDEX hi[1];     // Array of highest indexes of inner loops     
    NL_INDEX** hs;      // Array  of  highest  indexes of  segments of  inner loops

    NL_REAL epc = dCurveChordHeightTolerance;
    NL_REAL eps = dSurfaceChordHeightTolerance;
    NL_REAL tol = 1.0e-7;

    // work with copy of surface since it gets modified
    SmBSplineSurface* pBSS = new (*pContext) SmBSplineSurface(*pSrf);
    SmObjDelete sSurfCleanup(pBSS);
    sur = pBSS->GetOrCreateGwNurbPointer();

    // optional scale tolerances to size of minmax box
    if (eps < SM_EFF_ZERO)  //this is what triggers it
    {
        SmExtent3d sBBox;
        pSrf->CalculateBoundingBox(sBBox);
        eps = sBBox.GetSize().Length() / 300.0;
    }

    if (epc < eps)  // and for zero epc, use 1/2 surface tol
    {
        epc = eps / 2.0;
    }

    NL_STACKS S;
    N_InitNurbs(&S);

    // Now start setting up what we can
    hs = NULL;
    hi[0] = -1;
    cui = N_AllocArrayTripleCrvPtrs(0, &S);
    cui[0] = NULL;

    cuo = N_AllocArrayRealCrvPtrs(0, &S);
    ho[0] = 3;
    cuo[0] = N_AllocArrayCrvPtrs(3, &S);

    // Get constant parameter uv curves (lines) from surface
    NL_PARAMETER ul, ur, vb, vt;
    N_SrfGetParameterBounds(sur, &ul, &ur, &vb, &vt);

    // domain corner points: in CCW order.
    NL_POINT  P1;  P1.x = ul; P1.y = vb; P1.z = NL_NOZ;
    NL_POINT  P2;  P2.x = ur; P2.y = vb; P2.z = NL_NOZ;
    NL_POINT  P3;  P3.x = ur; P3.y = vt; P3.z = NL_NOZ;
    NL_POINT  P4;  P4.x = ul; P4.y = vt; P4.z = NL_NOZ;

    NL_CURVE crv1;
    N_CrvInitArrays(&crv1);
    N_CrvLineFrom2Pts(P1, P2, &crv1, &S);
    cuo[0][0] = &crv1;
    NL_CURVE crv2;
    N_CrvInitArrays(&crv2);
    N_CrvLineFrom2Pts(P2, P3, &crv2, &S);
    cuo[0][1] = &crv2;
    NL_CURVE crv3;
    N_CrvInitArrays(&crv3);
    N_CrvLineFrom2Pts(P3, P4, &crv3, &S);
    cuo[0][2] = &crv3;
    NL_CURVE crv4;
    N_CrvInitArrays(&crv4);
    N_CrvLineFrom2Pts(P4, P1, &crv4, &S);
    cuo[0][3] = &crv4;

    NL_PARAMETER* u, * v;
    NL_INDEX n, error;
    NL_INDEX** DT, * hd;

    error = N_TessTrimmedSrf(sur, cuo, cui, m, ho, hi, hs, epc, eps, tol, &u, &v, &n, &DT, &hd, &S);
    if (error)
    {
        N_EndNurbs(&S);
        SER(SM_ERR);
    }

    // Tessellation worked - get out triangles and dump to graphics
    NL_INDEX* alf, * bet, * gam;
    NL_INDEX numtri;
    if (N_TessGetTriangles(DT, hd, n, &alf, &bet, &gam, &numtri, &S) == NL_NO)
    {
        for (long kk = 0; kk <= numtri; kk++)
        {
            SmPoint2d sUV1(u[alf[kk]], v[alf[kk]]);
            SmPoint3d sPts[3];
            SmPoint2d sUVPts[3];
            SE(pBSS->EvaluatePoint(sUV1, sPts[0]));
            sUVPts[0] = sUV1;
            SmPoint2d sUV2(u[bet[kk]], v[bet[kk]]);
            SE(pBSS->EvaluatePoint(sUV2, sPts[1]));
            sUVPts[1] = sUV2;
            SmPoint2d sUV3(u[gam[kk]], v[gam[kk]]);
            SE(pBSS->EvaluatePoint(sUV3, sPts[2]));
            sUVPts[2] = sUV3;
            SmVector3d sNorms[3];
            SE(pBSS->EvaluateNormal(sUV1, TRUE, TRUE, sNorms[0]));
            SE(pBSS->EvaluateNormal(sUV2, TRUE, TRUE, sNorms[1]));
            SE(pBSS->EvaluateNormal(sUV3, TRUE, TRUE, sNorms[2]));

            if (bReverseNormals)
            {
                sNorms[0] = -sNorms[0];
                sNorms[1] = -sNorms[1];
                sNorms[2] = -sNorms[2];
            }

            // Output a triangle
            SER(rPolygonOutput.OutputPolygon(0, 3, sPts, sNorms, sUVPts, pBSS, NULL, pOptGfxSet));
        }
    }
    else // extraction failed
    {
        N_EndNurbs(&S);
        SER(SM_ERR);
    }

    N_EndNurbs(&S);

    return SM_SUCCESS;

} // end SmSurface::OutputPolygons

/*******************************************************************//**
PURPOSE:  Change knot and control Point values to extend the BSpline's
     Natural UVDomain to the specified parameter value by extending the
     range of one of the surface's boundary spans.  

NOTES:
  This method uses 'natural' extension: simply extend the domain of the
  boundary span to the specified parameter.  This method of surface extension
  adds no knots or control points, and leaves it C-infinity smooth where the
  original domain boundary was.  However, this can generate wildly changing
  surface shapes for large, or even moderate extensions.
  Use this function to increase the domain of a surface only by small amounts.

  One of the 4 possible boundary values is modified.  The input 
  eCrossSectionSurfParam value of SM_SP_U or SM_SP_V specifies
  whether a U or V boundary is modified.

  When the given parameter value, dNewUorVParam, is less than the current 
  UVDomain Min, the lower boundary is moved; when the value is larger
  than the current UVDomain max, the upper boundary is moved, else no
  action is taken.
  
  This function does not change the number of knots or Control Points.

  The output geometry over the originally defined domain is coincident with the input geometry
  but the parameterization may change slightly but more than reasonable sized tolerances.

  Callers of this method should delete all the UVTrimCurves associated
  with the surface being modified.
***********************************************************************/
SmStatus SmBSplineSurface::ExpandBoundarySpan
  (double dNewUorVParam,                    // in : desired new U or V NaturalDomain boundary value
   SmSurfParamType eCrossSectionSurfParam)  // in : SM_SP_U = setting a U boundary value
                                            //      SM_SP_V = setting a V boundary value
{
  // return value
  SmStatus RtnStatus = SM_ERR ;

  // Bspline locals
  SmExtent2d UVDomain = GetNaturalUVDomain() ;

  // N_SrfExtendByParamDist locals
  NL_SURFACE *surP = GetOrCreateGwNurbPointer();
  NL_FLAG     dflg =   (eCrossSectionSurfParam == SM_SP_U)
                  ? NL_UDIR
                  : NL_VDIR ;
  NL_FLAG     end  =   (   (dflg == NL_UDIR && dNewUorVParam > UVDomain.GetMax().x) 
                     || (dflg == NL_VDIR && dNewUorVParam > UVDomain.GetMax().y)) ? NL_END
                  : (   (dflg == NL_UDIR && dNewUorVParam < UVDomain.GetMin().x) 
                     || (dflg == NL_VDIR && dNewUorVParam < UVDomain.GetMin().y)) ? NL_START
                  : 0 ;
  
  // no work - specified param value contained within UVDomain
  if ( end == 0 )
    { return ( SM_SUCCESS ); }

  NL_FLAG     cont = NL_CMAX ;
  NL_SURFACE  surQ ; 
  N_SrfInitArrays(&surQ) ;
  NL_STACKS   SL ;  
  
  // Start NURBS
  SmNLibStackHandler sSH(&SL);

  // pass the call along
  NL_FLAG error = N_SrfExtendByParamDist(surP, dNewUorVParam, dflg, end, cont, &surQ, &SL, &SL) ;
  if ( error != NL_NO )
    { return SM_ERR ; }

  // verify that surP is the same size as surQ
  SmBoolean bSameSize =   (   surP->net->n == surQ.net->n
                           && surP->net->m == surQ.net->m
                           && surP->knu->m == surQ.knu->m
                           && surP->knv->m == surQ.knv->m)
                        ? TRUE
                        : FALSE ;
  SM_ASSERT(bSameSize) ;
  
  // when N_SrfExtendByParamDist worked as expected
  if(bSameSize) { // copy surQ values into surP
                  Notify(SM_NO_PRE_EDIT, this, SM_NO_GET_OWNER(this), NULL) ;
                  N_SrfCopy(&surQ, surP, &SL) ;
                  Notify(SM_NO_POST_EDIT, this, SM_NO_GET_OWNER(this), NULL) ;

                  RtnStatus = SM_SUCCESS ;
                }
  else          { // something went wrong in the N_SrfExtendByParamDist call - leave surP unmodified
                  RtnStatus = SM_ERR ;
                }    

  // give analytics a chance to update their nested STEP data
  RebuildSTEPFromNURBParameters() ;

  // all done
  SM_DUMP_AND_ASSERT2_VALID(this) ;
  return(RtnStatus);

} // end SmBSplineSurface::ExpandBoundarySpan

//
// Some local helper routines for the evaluators.
//

/*******************************************************************//**
PURPOSE: Try to find a direction for zero-length first derivatives.

NOTES: If rSu or rSv is shorter than dTol and is reset by
   this routine, the returned magnitude will be slightly greater than dTol.

   Returns TRUE iff rSu and rSv are longer than dTol on return.
***********************************************************************/
static SmBoolean sm_check_zero_surface_derivs
 (const SmPoint2d  &rUV,     // in :
  const SmExtent2d &rDomain, // in :
  const SmPoint3d  &rPt,     // NotUsed: in : pos of tgt eval - only used to set tolerance values
        SmVector3d &rSu,     // i/o: changed from zero length, if possible
        SmVector3d &rSv,     // i/o:
  const SmVector3d &rSuu,    // in :
  const SmVector3d &rSuv,    // in : mixed partial
  const SmVector3d &rSvv,    // in :
  const double      dScaledZero)    // in : pass in so it will be consistent.
{
  SM_REF1(rPt) ;
  SmBoolean bRetVal    = TRUE;

  double dNewLength = 1.1 * dScaledZero; // length of output vectors.
  double dLen;

  // The mixed partial (Suv) is the first derivative of both Su and Sv,
  // with respect to the other, and hence can be used for either one
  // if it's not zero.  Try that first.

  // If we're at the 'top' of the domain, i.e., the 'good' parameter is
  // entering a singularity instead of leaving it, then the degenerate
  // derivative is shrinking to zero, so its change is in the opposite
  // direction of its derivative.
  // If the derivative (of the derivative) is its u-derivative (which would
  // be the case when using Suv for Sv, or Suu for Su), then we check whether
  // the u value is at the top or bottom of its domain.
  SmBoolean bFlip = FALSE;

  if ( rSu.Length() < dScaledZero )
    {
      bRetVal = FALSE;

      // first check how Su is changing with v
      dLen = rSuv.Length();
      if ( dLen > dScaledZero )
        {
          rSu = rSuv * (dNewLength / dLen) ;
          bFlip = rUV.y > rDomain.GetVInterval().GetMid(); // v-deriv of Su
          bRetVal = TRUE;
        }
      else
        {
          // try change w.r.t. u
          dLen = rSuu.Length();
          if ( dLen > dScaledZero )
            {
              rSu = rSuu * (dNewLength / dLen) ;
              bFlip = rUV.x > rDomain.GetUInterval().GetMid(); // u-deriv of Su
              bRetVal = TRUE;
            }
        }

        if ( bRetVal == TRUE && bFlip )
        {
            rSu *= -1;
        }
    }

  if ( rSv.Length() < dScaledZero )
    {
      bRetVal = FALSE;

      // first check how Sv is changing with u
      dLen = rSuv.Length();
      if ( dLen > dScaledZero )
        {
          rSv = rSuv * (dNewLength / dLen) ;
          bFlip = rUV.x > rDomain.GetUInterval().GetMid(); // u-deriv of Sv
          bRetVal = TRUE;
        }
      else
        {
          // try change w.r.t. v
          dLen = rSvv.Length();
          if ( dLen > dScaledZero )
            {
              rSv = rSvv * (dNewLength / dLen) ;
              bFlip = rUV.y > rDomain.GetVInterval().GetMid(); // v-deriv of Sv
              bRetVal = TRUE;
            }
        }

        if ( bRetVal == TRUE && bFlip )
        {
            rSv *= -1;
        }
    }

  return bRetVal;

}  // end local sm_check_zero_surface_derivs


/*******************************************************************//**
PURPOSE: Evaluate the point and partial derivatives of a surface.

NOTES: 
   Zero first derivatives: if the flag bNonZeroTangents is passed in as True,
   then this routine find and will return a non-zero value for any zero derivative.
   Its magnitude will be set to 1.1 * dScaledZero, where dScaledZero is
   SM_EFF_ZERO * (1.0 + sOrigPnt.GetMaxDimension()), where sOrigPnt is the
   position of the surface evaluation at this uv.  This guarantees that
   the resulting vectors will always be ok in a call to Unitize().

   If this routine is unable to find a non-zero derivative when asked,
   it will return SM_ERR.


   Memory: You must allocate the array of output derivatives to be
   size of derivatives you expect.

Example:
   ULONG lHighestUDeriv = 2;  // 2nd derivative in U
   ULONG lHighestVDeriv = 1;  // 1st derivative in V 
   SmVector3d sDerivs[lHighestUDeriv+1][lHighestVDeriv+1];
   Evaluate(..,lHighestUDeriv,lHighestVDeriv,TRUE,TRUE,TRUE,sDerivs[0]);

   SmPoint3d sPoint = sDerivs[0][0];
   SmVector3d sDU   = sDerivs[1][0];
   SmVector3d sDV   = sDerivs[0][1];

   // Following one not given if bOnlyUpperHalf == TRUE when highest = 1 and 1
   SmVector3d sDUV  = sDerivs[1][1];  

   SmVector3d sDUU  = sDerivs[2][0];
   SmVector3d sDVV  = sDerivs[0][2];

   // Following three not given if bOnlyUpperHalf == TRUE when highest = 2 and 2
   SmVector3d sDUUV = sDerivs[2][1];    
   SmVector3d sDUVV = sDerivs[1][2];
   SmVector3d sDUUVV = sDerivs[2][2];

   // This can go on until you get to the limit of the derivatives as follows:
   SmVector3d sDUUUUVVVV = sDerivs[4][3]; 

   // The maximum derivative obtainable is MAX_DERIV which equals 5.
   // SmVector3d sMax[MAX_DERIV+1][MAX_DERIV+1] is the largest matrix 
   // we can evaluate on a surface.

Note: bOnlyUpperHalf applies only if lHighestUDerv == lHighestVDeriv.
In fact, if sDerivs is set up with lHighestUDerv != lHighestVDeriv
and bOnlyUpperHalf is passed in TRUE, memory will be written incorrectly
and crashes could result.  This routine has been set up to return an
error in this case.

***********************************************************************/
SmStatus SmBSplineSurface::Evaluate
  (const SmPoint2d & crUV,             // in : param value to evaluate
   ULONG             lHighestUDeriv,   // in : number of U derivatives
   ULONG             lHighestVDeriv,   // in : number of V derivatives to compute
   SmBoolean         bUFromLeft,       // in : if P is on U interval boundary
                                       //      TRUE  = evaluate P in upper interval where P is on the left of the interval
                                       //      FALSE = evaluate P in lower interval where P is on the right of the interval
   SmBoolean         bVFromLeft,       // in : if P is on V interval boundary
                                       //      TRUE  = evaluate P in upper interval where P is on the left of the interval
                                       //      FALSE = evaluate P in lower interval where P is on the right of the interval
   SmBoolean         bOnlyUpperHalf,   // in : TRUE=compute upper half of matrix only
                                       //      ex. 1,1 = [D  Du] 2,2 = [D    Du    Duu] where -- = an untouched memory value 
                                       //                [Dv --]       [Dv   Duv   ---]           (the memory has to be allocated)
                                       //                              [Dvv  ---   ---]
   SmVector3d      * aDerivatives,     // out: matrix of evaluations values
                                       //      sized:[lHighestUDeriv+1][lHighestVDeriv+1]
                                       //      2d organized: [D    Du    Duu    Duuu    Duuuu   ]  (the same no matter the value of)
                                       //                    [Dv   Duv   Duuv   Duuuv   Duuuuv  ]  (  bOnlyUpperHalf               )
                                       //                    [Dvv  Duvv  Duuvv  Duuuvv  Duuuuvv ]
                                       //                    [Dvvv Duvvv Duuvvv Duuuvvv Duuuuvvv]
                                       //      1d organized: [D, Dv, Dvv, Dvvv,.. Du, Duv, Duvv, Duvvv,.. Duu, Duuv, Duuvv, Duuvvv,...]
                                       //                     Du, Duv, Duvv, Duvvv,.. 
                                       //                     Duu, Duuv, Duuvv, Duuvvv,...]
  SmBoolean          bNonZeroTangents, // in : TRUE = replace zero tangent vectors with properly oriented tol sized vectors
                                       //      FALSE= return exact tangent values
                                       //      note: Surprisingly TRUE is the common choice because most tangent uses
                                       //            are for their direction (Binorm, SurfNorm comps), but when the 
                                       //            tangent is being used for its magnitude (like an arc-length comp)
                                       //            then set this to FALSE.
                                       //      default:[TRUE]
  SmBoolean          bDoZeroSampling)  // in : for internal use only, always set to TRUE, default:[TRUE]
 const
{
#ifdef SM_DEBUG_CODE
//static const SmBSplineSurface *pLast = NULL ;
//  if(!IsBounded()) { pLast = this ; }
SmBoolean bDebugMe = FALSE ;
  // draw 
  if(bDebugMe)
    {
      SM_DUMP_AND_ASSERT_VALID(this) ;

      SmFace *pFace = (SmFace *)GetFace() ;
      SmBrep *pBrep =   pFace ? pFace->GetBrep(): NULL ;

      smgfx_Erase() ;
      smgfx_SetLook( 1, 2, 0, 0, 1 ); if(pBrep) { pBrep->Draw( TRUE ); sm_GraphicsLoop(); }
      smgfx_SetLook(1,2, 0,1,1) ; this->DrawUV() ; sm_GraphicsLoop() ;
      smgfx_SetLook( 1, 2, 0, 0, 0 ); if(pFace) { pFace->Draw( SM_DM_CROSSHATCH ); sm_GraphicsLoop(); }
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // allow pressing the escape to exit at every 10000 evaluates
    /*
  static ULONG lCounter = 0;
  ULONG i;
  lCounter++ ;
  if(lCounter % 10000 == 0 && smos_ExcapeCallback()) 
    {
      return SM_ERR;
    }
    */

  // check input - bOnlyUpperHalf can be TRUE only if deriv requests are equal.
  if ( bOnlyUpperHalf && ( lHighestUDeriv != lHighestVDeriv ) )
    {
      SER(SM_ERR_INVALID_INPUT);
    }

  // check input - invalid to ask for 
  //      if ( lHighestUDeriv > NL_MAXDEG ) SER( SM_ERR_INVALID_INPUT ); // GWC: should be NL_MAXDER?
  //      if ( lHighestVDeriv > NL_MAXDEG ) SER( SM_ERR_INVALID_INPUT ); // GWC: should be NL_MAXDER?

  if ( lHighestUDeriv > NL_MAXDER ) SER( SM_ERR_INVALID_INPUT ); 
  if ( lHighestVDeriv > NL_MAXDER ) SER( SM_ERR_INVALID_INPUT ); 

  // When appropriate, clamp input parameters to surface domain
  SmPoint2d  sUV( crUV );
  SmExtent2d sNaturalUVDomain = this->GetNaturalUVDomain();
  if(   m_eBSplineSurfaceForm != SM_SF_POLYNOMIAL 
     && !sNaturalUVDomain.ContainsPoint2d(sUV)
     && !IsOutOfBoundsEnabled())                 // GWC:MODIFIED OUTOFBOUNDS, default:[FALSE]
    {
      sUV = sNaturalUVDomain.ClampPoint2d(sUV);
    }

  // Setup for NLib call: 
  NL_FLAG uflg = (bUFromLeft) ? NL_LEFT : NL_RIGHT ;
  NL_FLAG vflg = (bVFromLeft) ? NL_LEFT : NL_RIGHT ;
  NL_FLAG err  = 0;

  //   Build SD, the 2d array of evaluate values, as an array of NL_POINT pointers to pointers.
  //   Assign SD pointers into aDerivatives.
  NL_POINT  *SD[NL_MAXDER+1];
  for (ULONG i=0; i<=lHighestUDeriv; i++) 
    {
      // cast here is OK because 
      // an NL_POINT and an SmVector3d are both three doubles in the same order (x,y,z).
      SD[i] = SM_REINTERPRET_CAST(NL_POINT *,&aDerivatives[i*(lHighestVDeriv+1)]);
    }

  // pass the call along for forms == SM_SF_POLYNOMIAL
  if ( m_eBSplineSurfaceForm == SM_SF_POLYNOMIAL )
    {
      err = N_SrfPowerBasisEvalDerivs((NL_SURFACE*)m_pNurb, 
                                      sUV.x, 
                                      sUV.y, 
                                      (NL_FLAG)bOnlyUpperHalf,
                                      lHighestUDeriv, 
                                      lHighestVDeriv, 
                                      SD);
    }
  else // for all other forms, pass the call along
    {
      err = N_SrfDerivs((NL_SURFACE*)m_pNurb, 
                        sUV.x, 
                        sUV.y,
                        uflg, 
                        vflg, 
                         (NL_FLAG)bOnlyUpperHalf,
                        lHighestUDeriv, 
                        lHighestVDeriv, 
                        SD);
    }

  // check for errors
  if(err) 
    { NL_SER(err) ; }

//cbi: decided not to do this (Telecon 5/29/12).
//  // Check for Inf/NaN etc.  They return False for any comparison.
//  if ( aDerivatives[0][0] != aDerivatives[0][0] )
//    { SER( SM_ERR_INVALID_INPUT ); }

  // If we don't have to check for zero first derivatives, we're done.
  if ( ! bNonZeroTangents                       ) { return SM_SUCCESS; }
  if ( lHighestUDeriv < 1 && lHighestVDeriv < 1 ) { return SM_SUCCESS; }


  // Check for zero first derivatives.

  // position and tangents
  //    watch out for aDerivatives boundary violations,
  //    for now: when a derivative is missing - just assign the first derivative that is available to both values.
  SmPoint3d  sOrigPnt = aDerivatives[0];
  SmVector3d sOrigDU  = (lHighestUDeriv >= 1) ? aDerivatives[lHighestVDeriv+1] : aDerivatives[1] ;
  SmVector3d sOrigDV  = (lHighestVDeriv >= 1) ? aDerivatives[1]                : aDerivatives[lHighestVDeriv+1] ;

  // tangent sizes
  double sOrigDULen = sOrigDU.Length() ;
  double sOrigDVLen = sOrigDV.Length() ;

  double dDerivLim = GetDerivativeLimit( sOrigPnt );

  // test
  SmBoolean bZeroDU = sOrigDULen < dDerivLim ;
  SmBoolean bZeroDV = sOrigDVLen < dDerivLim ;

  // no work -  no zero tangents to fix
  //    when both original tangents are larger than tolerance.
  //        This used to quit when tangents were short but the
  //             ratio of their lengths was in tolerance.
  //        I've removed that to let the trick attempt to find a nonZeroTangent value
  //             for all short enough tangents.  GWC 

  if( bZeroDU == FALSE && bZeroDV == FALSE )
    { return SM_SUCCESS ; }


  // arrive here to kluge a short properly-directed vector for a zero 1st derivative.
  //   Note: the magnitude of a kluged derivative doesn't matter
  //         because it's not correct anyway.  (It's supposed to be zero.)
  //         So just try to find its direction, and return something short.

  // When working with just one zero tangent vector,
  //    Suv works best for both Su and Sv, else use their respective 2nd derivs.

  // check for a point with two zero tangents
  //  I've tried to anticipate this but we may need to rethink the behavior
  //   in response to this rare case.  I suspect we may not get a good solution
  //   without inspecting nearby points.
  //   Example 1 :  a singular side next to a repeated endPoint row
  //     solution:  get nonZeroTangent values in the repeated endPoint direction
  //                at two points - use those to estimate sSuv (the 2nd derivative)
  //                and then use sSuv as the estimate for the tangent in the
  //                the degenerate direction.
  //  Example 2 : a pair of singular sides meeting at a corner
  //    solution:  get estimates from nearby points.  All the derivatives at
  //               the corner will be zero and this trick won't work here.
  //     I've not coded up anything as elaboate as mentioned above - in practice
  //     we may need to do that. For now, I'll output a message so that if
  //     a failed evaluate causes a bigger failure we'll have a hint of where to
  //     look first.
  SmBoolean bTwoZeroTangents = bZeroDU && bZeroDV ;

  // when not enough derivatives were asked for - recurse to this function
  //   note: we may have to use the recursion trick when bTwoZeroTangents is TRUE
  //         but we'll ignore that for now.
  if ( lHighestUDeriv < 2 || lHighestVDeriv < 2 )
    {
      // recurse values
      SmVector3d aReDer[9] ;

      // pass the call along
      SER(Evaluate(crUV, 2, 2, bUFromLeft, bVFromLeft, TRUE, aReDer, TRUE)) ;

      // save the kluged tangent values
      if(lHighestUDeriv >=1) { aDerivatives[lHighestVDeriv+1] = aReDer[3]; }
      if(lHighestVDeriv >=1) { aDerivatives[1]                = aReDer[1]; }
      
      // all done
      return(SM_SUCCESS) ;

    } // end recursion branch when asked for less than 2 derivatives


  // Ok, now we have at least two derivatives.
  // This is where we deal with the zero-derivative problem.

  ULONG lNV = lHighestVDeriv;  // shorthand
  SmBoolean bOk = FALSE;
  bOk = sm_check_zero_surface_derivs(
              sUV,                   // in :
              sNaturalUVDomain,      // in :
              aDerivatives[  0    ], // in : postion: used only for tolerance values
              aDerivatives[  lNV+1], // i/o: Su: changed from zero length, if possible
              aDerivatives[  1    ], // i/o: Sv: ditto
              aDerivatives[2*lNV+2], // in : Suu
              aDerivatives[  lNV+2], // in : Suv
              aDerivatives[  2    ], // in : Svv
              dDerivLim
             );


#ifdef SM_DEBUG_CODE
  if ( bDebugMe ) {
      SmVector3d sSu( aDerivatives[lHighestVDeriv+1] );
      SmVector3d sSv( aDerivatives[1] );
      sSu.Unitize();
      sSv.Unitize();
      // Make the non-degenerate deriv bigger.
      if ( bZeroDU ) { sSu *= 2.0; }
      if ( bZeroDV ) { sSv *= 2.0; }
      smgfx_SetLook( 1,2, 0,0,1 ); sSu.Draw( aDerivatives ); sm_GraphicsLoop();
      smgfx_SetLook( 1,2, 1,0,1 ); sSv.Draw( aDerivatives ); sm_GraphicsLoop();
      sm_GraphicsLoop();
  }
#endif

  // all done?
  if ( bOk )
    { return SM_SUCCESS; }


  // arrive here when:
  //   need to kluge a direction for a zero 1st derivative, 
  //   and getting the kluged 1st der direction from 2nd derivatives failed.

  // bZeroDV and/or bZeroDU are TRUE - work until they are both FALSE

  // Try finding a nonZeroTangent value with a more brute-force method.
  // Step around and try to find a good one nearby.

  // The problem derivatives should be at the ends of the domains.
  // In any case, we want to step towards the center of the domain.
  // Set step directions to + for the low end and - for the high end.
  double dStepDirU = ( sUV.x > sNaturalUVDomain.GetUInterval().GetMid() ) ? -1.0 : 1.0;
  double dStepDirV = ( sUV.y > sNaturalUVDomain.GetVInterval().GetMid() ) ? -1.0 : 1.0;

  // recurse values
  SmVector3d aReDer[4] ;
  double dStep = SM_ZONE_TOL_3D/10.0;  // works well in practice.

  // for larger and larger step off distances
  while(bDoZeroSampling && dStep < 0.1)
    {
      // increase the step off distance
      dStep = dStep * 10.0;

      // for this step size: try 5 directions; last 4 are diagonals.
      SmPoint2d sTestUV[5];  

      // Make the first step go in the most likely direction:
      // direction of good deriv, towards center of surface.
      // This should have the smallest effect on things.
      if ( sOrigDULen < sOrigDVLen )   
        {
          // step in v                                                
          sTestUV[0].x = sUV.x;                                     
          sTestUV[0].y = sUV.y + dStepDirV * dStep;
        }
      else // Sv < Su
        {
          // step in u
          sTestUV[0].x = sUV.x + dStepDirU * dStep ;
          sTestUV[0].y = sUV.y;
        }
  
      // If that doesn't work, just try all four different directions
      sTestUV[1].x = sUV.x + dStep;
      sTestUV[1].y = sUV.y + dStep;
  
      sTestUV[2].x = sUV.x - dStep;
      sTestUV[2].y = sUV.y - dStep;
  
      sTestUV[3].x = sUV.x - dStep;
      sTestUV[3].y = sUV.y + dStep;
  
      sTestUV[4].x = sUV.x + dStep;
      sTestUV[4].y = sUV.y - dStep;
  
      // for all 5 sample directions at this step size
      for(ULONG i=0;i<5;i++)
        {
          sTestUV[i]    = sNaturalUVDomain.ClampPoint2d(sTestUV[i]);
          // FALSE: don't ask it to try to do these adjustments,
          // just take what's naturally there.
          // Also, we need only one derivative from it.
          SmStatus sRtn = Evaluate(sTestUV[i], 1, 1, bUFromLeft, bVFromLeft, TRUE, aReDer, FALSE, FALSE) ;

          if ( sRtn != SM_SUCCESS)
            { continue; }

          // Save the nearby tangent values, if they're good enough.
          if (   aReDer[2].Length() >= dDerivLim
              && aReDer[1].Length() >= dDerivLim )
            {
              // Found a good result.
              // Set magnitudes of degenerate derivatives to the standard 1.1 * dDerivLim.
              if ( bZeroDU )
                {
                  aReDer[2].Unitize();
                  aReDer[2] *= ( 1.1*dDerivLim );
                  aDerivatives[lHighestVDeriv+1] = aReDer[2];
                }
              if ( bZeroDV )
                {
                  aReDer[1].Unitize();
                  aReDer[1] *= ( 1.1*dDerivLim );
                  aDerivatives[1] = aReDer[1];
                }

              // all done
              return(SM_SUCCESS) ;
            }

        } // end iter all test points - recursion branch
    } // end iter all larger step sizes

  if(bTwoZeroTangents)
    {                
      SM_DBG_WARN(_T("SM_ERR in SmBSplineSurface::Evaluate - evaluating a surface point with two zero tangents")) ;
    }

  // If we make it to here we are unable to get both of the first
  // derivatives to a non-zero size.  Return an error.
  return SM_ERR;

    //cbi: // end asked to fix nonZeroTangents check

  // all done
  return(SM_SUCCESS) ;

 //      // obsolete sampling section - replaced with recursive calls
 // if(bNonZeroTangents && (lHighestUDeriv > 0 || lHighestVDeriv > 0))
 // { . . .
 //     { . . .
 //           // bZeroDV and/or bZeroDU are TRUE - work until they are both FALSE
 //      
 //           // Try finding a nonZeroTangent value with a more brute-force method.
 //           // Step around and try to find a good one.
 //      
 //           // simplified upcoming loop by making recursive calls
 //           // Make a new array for stepped evals, and set it up with SD as before.
 //           SmVector3d aOffsetEval[(MAXDER+1)*(MAXDER+1)];
 //           for (i=0; i<=lHighestUDeriv; i++) 
 //             {
 //               SD[i] = SM_REINTERPRET_CAST( NL_POINT *, &aOffsetEval[ i*(lHighestVDeriv+1)] );
 //             }
 //           
 //           // For use in loop:
 //           SmVector3d sDUNew, sDVNew;
 //           SmExtent2d sDomain = this->GetNaturalUVDomain();
 //           SmPoint2d sMidUV = sDomain.GetMin()  +  0.5 * sDomain.GetSize();
 //           // sample the function
 //           // for larger and larger step off distances
 //           while (dStep < 0.1)
 //             {
 // 
 //                   sTestUV[i] = sDomain.ClampPoint2d(sTestUV[i]);
 //                   if (m_eBSplineSurfaceForm == SM_SF_POLYNOMIAL) 
 //                     {
 //                       // sample the function for SM_SF_POLYNOMIAL forms
 //                       NL_SER(N_SrfPowerBasisEvalDerivs((SURFACE*)m_pNurb, 
 //                                                        sTestUV[i].x, 
 //                                                        sTestUV[i].y,
 //                                                        bOnlyUpperHalf, 
 //                                                        lHighestUDeriv, 
 //                                                        lHighestVDeriv,
 //                                                        SD ));
 //                     }
 //      
 //                   else  // sample the function for all other forms
 //                     {
 //                       NL_SER(N_SrfDerivs((SURFACE*)m_pNurb,
 //                                          sTestUV[i].x, 
 //                                          sTestUV[i].y, 
 //                                          uflg, 
 //                                          vflg,
 //                                          bOnlyUpperHalf, 
 //                                          lHighestUDeriv, 
 //                                          lHighestVDeriv,
 //                                          SD ));
 //                     }
 //      
 //                   // 1st tangent derivatives and their sizes
 //                   sDUNew = aOffsetEval[lHighestVDeriv+1];
 //                   sDVNew = aOffsetEval[1];
 //      
 //                   double dUSize2 = sDUNew.GetMaxDimension() ;
 //                   double dVSize2 = sDVNew.GetMaxDimension() ;
 //                   double dRatio2 = ( sOrigDULenSq < sOrigDVLenSq ) 
 //                                   ? (dVSize2 > SM_EFF_ZERO) ? (dUSize2 / dVSize2) : 1.0 
 //                                   : (dUSize2 > SM_EFF_ZERO) ? (dVSize2 / dUSize2) : 1.0 ;
 //      
 //                   // fix short Du
 //                   if( bZeroDU && sDUNew.LengthSquared() > dTolSq)
 //                     { sDUNew.Unitize();
 //                       sDUNew *= 1.1*dTol;  // no need to flip this with dFlipUDeriv because it's an actual Su from a nearby point
 //                       aDerivatives[lHighestVDeriv+1] = sDUNew;    // Su
 //                       bZeroDU = FALSE ;
 //                     }
 //      
 //                   if( bZeroDV && sDVNew.LengthSquared() > dTolSq)
 //                     { sDVNew.Unitize();
 //                       sDVNew *= 1.1*dTol;  // no need to flip this with dFlipVDeriv because it's an actual Su from a nearby point
 //                       aDerivatives[1] = sDVNew;    // Sv
 //                       bZeroDV = FALSE ;
 //                     }
 //      
 //                    // if we're done
 //                    if(bZeroDU == FALSE && bZeroDV == FALSE)
 //                      {
 //                       // we're done
 //                       return SM_SUCCESS;
 //                      }
 //                 } // end itering all 5 samples looking for a good eval point
 //      
 //             } // end while dStep < 1.0
 //      
 //           if(bTwoZeroTangents)
 //             {                
 //               SM_DBG_WARN(_T("SM_ERR in SmBSplineSurface::Evaluate - evaluating a surface point with two zero tangents")) ;
 //             }
 //      
 //           // If we make it to here we are unable to get both of the first
 //           // derivatives to a non-zero size.  Return an error.
 //           return SM_ERR;
 //      
 //         } // end bNonZeroTangents when evaluating tangents check
  //      
  //      // all done
  //      return(SM_SUCCESS) ;

} // end SmBSplineSurface::Evaluate

/*******************************************************************//**
PURPOSE: Evaluate the NURBS directly and do not use any derivative
    correction techniques by stepping off.

NOTES: 
***********************************************************************/
SmStatus SmBSplineSurface::EvaluateSimple
  (const SmPoint2d & crUV,    // in : target surface point
   ULONG lHighestUDeriv,      // in : Requested highest U derivative
   ULONG lHighestVDeriv,      // in : Requested highest V derivative
   SmBoolean bUFromLeft,      // in : if P is on U interval boundary
                              //      TRUE  = evaluate P in upper interval where P is on the left of the interval
                              //      FALSE = evaluate P in lower interval where P is on the right of the interval
   SmBoolean bVFromLeft,      // in : if P is on V interval boundary
                              //      TRUE  = evaluate P in upper interval where P is on the left of the interval
                              //      FALSE = evaluate P in lower interval where P is on the right of the interval
   SmBoolean bOnlyUpperHalf,  // in : TRUE=compute upper half of matrix only
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
   // get surface context
   SmContext *pContext = SM_CONST_CAST(SmContext*,GetContext());
   NER(pContext);  

   // check input
   if (lHighestUDeriv > NL_MAXDEG) SER(SM_ERR_INVALID_INPUT);
   if (lHighestVDeriv > NL_MAXDEG) SER(SM_ERR_INVALID_INPUT);

   // locals
   SmPoint2d sUV = crUV; // needed so that &crUV can be same memory as aDerivatives
   if (m_eBSplineSurfaceForm != SM_SF_POLYNOMIAL) 
     {
       SmExtent2d sNaturalUVDomain = this->GetNaturalUVDomain();
       if (!sNaturalUVDomain.ContainsPoint2d(sUV)) 
         {
           sUV = sNaturalUVDomain.ClampPoint2d(sUV);
         }
     }

   // initialize the NLIB output array pointers
   NL_POINT  *SD[NL_MAXDEG+1];
   for (ULONG i=0; i<=lHighestUDeriv; i++) 
     {
       // We are a little tricky casting here because we know
       // that a NL_POINT  and a SmVector3d are both three doubles
       // in the same order (x,y,z).
       SD[i] = SM_REINTERPRET_CAST(NL_POINT *,&aDerivatives[i*(lHighestVDeriv+1)]);
     }

   // set NLIB arguments
   NL_FLAG uflg = bUFromLeft ? NL_LEFT : NL_RIGHT;
   NL_FLAG vflg = bVFromLeft ? NL_LEFT : NL_RIGHT;

   // make the NLIB Call
   if (m_eBSplineSurfaceForm == SM_SF_POLYNOMIAL) 
     {
       NL_SER(N_SrfPowerBasisEvalDerivs((NL_SURFACE*)m_pNurb, sUV.x, sUV.y, (NL_FLAG)bOnlyUpperHalf,
                       lHighestUDeriv, lHighestVDeriv, SD));
     }
   else 
     {
       NL_SER(N_SrfDerivs((NL_SURFACE*)m_pNurb, sUV.x, sUV.y,
                       uflg, vflg, (NL_FLAG)bOnlyUpperHalf,
                       lHighestUDeriv, lHighestVDeriv, SD));
     }


   // all done
   return SM_SUCCESS;

} // SmBSplineSurface::EvaluateSimple

/*******************************************************************//**
PURPOSE: Given a parametric point on a surface evalute the corresponding
    Euclidian point.

NOTES: 
***********************************************************************/
SmStatus SmBSplineSurface::EvaluatePoint
 (const SmPoint2d & crUV,    // in : Target Nurb UV Point
  SmPoint3d       & rPoint)  // out: Resulting Image Point
 const
{
  // check input
#ifdef SM_DEBUG_CODE
//static const SmBSplineSurface *pLast = NULL ;
//        SM_ASSERT_MSG((pLast == this || IsBounded()), 
//                       _T("Err: Using BSpline evaluator on infinite surface")) ;
//  if(!IsBounded()) { pLast = this ; }
#endif                  

  // every 10,000 evals - give the escape key a check
    /*
  static ULONG lCounter = 0;
  lCounter ++;
  if (lCounter % 10000 == 0) 
    {
      if (smos_ExcapeCallback()) 
        {
          return SM_ERR;
        }
    }
    */
  // clamp DomainPoints (when OutOfBounds evals are not enabled)
  // Note: there was a typo here, in which the actual calls used
  // crUV instead of sUV, i.e., the clamping didn't actually happen.
  // However, 'fixing' it makes things worse, so don't clamp.
  // (In which case, we don't need sVU at all.)
  // [bd, 05Jan06; regressions: Two Tori, and my_shell_demo iters 0, 3, 6]
#if 0  // (don't clamp)
  SmPoint2d sUV = crUV;
  if (m_eBSplineSurfaceForm != SM_SF_POLYNOMIAL) 
    {
      SmExtent2d sNaturalUVDomain = this->GetNaturalUVDomain();
      if (   !sNaturalUVDomain.ContainsPoint2d(sUV)
          && !IsOutOfBoundsEnabled()) // GWC:MODIFIED OUTOFBOUNDS
        {
          sUV = sNaturalUVDomain.ClampPoint2d(sUV);
        }
    } 
#endif  // (don't clamp)

  
  // pass the eval call along     
  NL_FLAG flg = NL_LEFT;
  NL_POINT  C;
  NL_FLAG err = 0;
  if (m_eBSplineSurfaceForm == SM_SF_POLYNOMIAL) 
    {
      err = N_SrfPowerBasisEvalPt((NL_SURFACE*)m_pNurb,crUV.x,crUV.y,&C);
    }
  else 
    {
      err = N_SrfEvalPt((NL_SURFACE*)m_pNurb,crUV.x,crUV.y,flg,flg,&C);
    }
  if ( err != 0) 
      SER(SM_ERR);

  // set output
  COPY_XYZ(C,rPoint);
  return SM_SUCCESS;

} // end SmBSplineSurface::EvaluatePoint

/*******************************************************************//**
PURPOSE: Move Control points in two surfaces so that the surfaces 
         meet across a G1 common boundary.

NOTES: 
   The surfaces may be the same surfaces.
     In that case, the WhichEdge flags must be consistent:
     one Min, one Max, same U or V.

   Currently this does G1 only.

   The surfaces must have the same number of control points along the
   common edge.  This routine is intended for surfaces where all of the
   control points from the two surfaces coincide along the edge, but that
   is not checked for, and the average positions are used.  For a single
   closed surface, or two surfaces of the same degree, if the edge curves
   do match, then the control points must be coincident, for nonrational
   surfaces.

   Input argument dPreserveEndFactor: Suggested values from 0.0 to 1.0.
   (Value may be out of this range, it is not checked.  But if it's not in
   this range, the resulting surfaces shapes will probably not be what you want.)
   If 1.0, the position of the join point will be preserved, and the surfaces
   around the join point will be pushed out to be G1; all of the control point
   movement will be taken up by the two adjacent points.
   If 0.0, then all of the point move will be at the join point.
   At values in between, the control point shifts will be apportioned
   between he join point and the two adjacent points.
   If the join point can be moved, the result is generally nicer surfaces.

   This works by control point manipulation, and moves only the seam point
   and the first adjacent rows on both sides.  As such, the result will be
   highly dependent on the control point distribution.  The result will
   indeed be G1, but if the control points are dense, the change will be
   very localized; if the crease is substantial, the result probably
   won't look nice.
***********************************************************************/
SmStatus SmBSplineSurface::SmoothJoin
(
  SmBSplineSurface * pSurf1,              // in : Input surface1
  SmSurfParamType    eWhichEdge1,         // in : SM_SP_UMIN, _VMIN, _UMAX, or _VMAX.
  SmBSplineSurface * pSurf2,              // in : Input surface2  
  SmSurfParamType    eWhichEdge2,         // in : SM_SP_UMIN, _VMIN, _UMAX, or _VMAX.
  double             dPreserveEndFactor   // In: see Usage Notes.  Default 0.5.
)
{
  // For easier notation:
  SmSurfParamType eCrossDir1 =   ( eWhichEdge1 == SM_SP_UMIN || eWhichEdge1 == SM_SP_UMAX )
                               ? SM_SP_U : SM_SP_V;
  SmSurfParamType eCrossDir2 =   ( eWhichEdge2 == SM_SP_UMIN || eWhichEdge2 == SM_SP_UMAX )
                               ? SM_SP_U : SM_SP_V;
  SmSurfParamType eSeamDir1 = (eCrossDir1==SM_SP_U) ? SM_SP_V : SM_SP_U;
  SmSurfParamType eSeamDir2 = (eCrossDir2==SM_SP_U) ? SM_SP_V : SM_SP_U;
  SmBoolean       bAtStart1 =    ( eWhichEdge1 == SM_SP_UMIN || eWhichEdge1 == SM_SP_VMIN )
                               ? TRUE : FALSE;
  SmBoolean       bAtStart2 =    ( eWhichEdge2 == SM_SP_UMIN || eWhichEdge2 == SM_SP_VMIN )
                               ? TRUE : FALSE;
                  
  // Grab the relevant control points.
  ULONG lNumPts = pSurf1->GetNumberControlPoints( eSeamDir1 );
  if ( pSurf2->GetNumberControlPoints( eSeamDir2 ) != lNumPts )
    { SER( SM_ERR_INVALID_INPUT); }


  SmControlPointFormType eForm1 = ( pSurf1->IsRational() ) ? SM_CP_EUCLIDIAN_RATIONAL : SM_CP_NON_RATIONAL;
  SmControlPointFormType eForm2 = ( pSurf2->IsRational() ) ? SM_CP_EUCLIDIAN_RATIONAL : SM_CP_NON_RATIONAL;

  // At each point along the join, we'll work with three points, just call them sP1, sP2, and sP3.
  //   sP2 is the common end point, sP1 is the adjacent point in surface 1,
  //   and sP3 is the adjacent point in surface 2.
  // When we're finished, these will be collinear.
  // We move sP2 onto the line segment connecting sP1 and sP3, or move sP1 and
  // sP3 so that the line segment contains sP2, or some combination of the two,
  // depending on dPreserveEndFactor.
  // But, where on the line segment should sP2 end up at?
  // A first thought might be to drop sP2 to that line segment, but that
  // could cause problems if, for instance, it dropped outside the
  // segement from sP1 to sP3.  We want to preserve, generally, its
  // position between sP1 and sP3.  So, put it on the interior of the
  // line segment according to the relative distances | sP1-sP2 | and
  // | sP2 - sP3 |.
  //
  // For surfaces, though, each point triplet being collinear is not sufficient
  // for G1.  A sufficient condition is that collinearity, plus the distance
  // ratios all being the same: the relative position of sP2 between sP1 and sP3.
  // So, what ratio should we use?  The best result would probably be an average
  // of all triples along the edge.  Unfortunately, we have to loop over the
  // whole seam to do that, and since most of the body of the loop is just
  // figuring out the indexing, we pretty much just have to do the whole
  // loop twice.  But I think it's worth it (for the shape), so here we go.

  SmPoint3d sP1, sP2, sP3, sTempPt;
  double    dW1, dW2, dW3, dTempWt;

  // Get number of points in the other direction, to get inidices of high-end points.
  ULONG lNumPts1 = pSurf1->GetNumberControlPoints( eCrossDir1 );
  ULONG lNumPts2 = pSurf2->GetNumberControlPoints( eCrossDir2 );
  ULONG lIdx1, lIdx2;

  // First run down the seam, getting an average fraction to use.
  ULONG ii;
  double dFrac = 0.0;  // We'll accumulate here, and average.

  for ( ii = 0; ii < lNumPts; ii++ )
    {
      // Actually, in this pass, we can set the end points to be the midpoint
      // of the given end points, besides getting the distance fraction.

      lIdx1 = ( bAtStart1 ) ? 0 : lNumPts1-1;
      lIdx2 = ( bAtStart2 ) ? 0 : lNumPts2-1;

      if ( eCrossDir1 == SM_SP_U ) { SER( pSurf1->GetControlPoint( eForm1, lIdx1, ii, sP2, dW2 )); }
      else                         { SER( pSurf1->GetControlPoint( eForm1, ii, lIdx1, sP2, dW2 )); }

      if ( eCrossDir2 == SM_SP_U ) { SER( pSurf2->GetControlPoint( eForm2, lIdx2, ii, sTempPt, dTempWt )); }
      else                         { SER( pSurf2->GetControlPoint( eForm2, ii, lIdx2, sTempPt, dTempWt )); }

      sP2 = ( sP2 + sTempPt ) / 2.0;
      dW2 = ( dW2 + dTempWt ) / 2.0;

      // Now the neighboring points.

      lIdx1 = ( bAtStart1 ) ? 1 : lNumPts1-2;
      lIdx2 = ( bAtStart2 ) ? 1 : lNumPts2-2;

      if ( eCrossDir1 == SM_SP_U ) { SER( pSurf1->GetControlPoint( eForm1, lIdx1, ii, sP1, dW1 )); }
      else                         { SER( pSurf1->GetControlPoint( eForm1, ii, lIdx1, sP1, dW1 )); }

      if ( eCrossDir2 == SM_SP_U ) { SER( pSurf2->GetControlPoint( eForm2, lIdx2, ii, sP3, dW3 )); }
      else { SER( pSurf2->GetControlPoint( eForm2, ii, lIdx2, sP3, dW3 )); }

      // And get the distance ratio.

      double dLen1 = sP2.DistanceBetween( sP1 );
      double dLen2 = sP2.DistanceBetween( sP3 );
      dFrac += dLen1 / ( dLen1 + dLen2 );

    } // end loop getting average fraction to use.

  dFrac /= lNumPts;  // Got the average.


  // Now run down each point along the seam again, doing the
  // 3-point algorithm with the fraction we just found.

  for ( ii = 0; ii < lNumPts; ii++ )
    {
      // The center points (sP2) have been set to be the same,
      // so we only have to grab one of them.

      lIdx1 = ( bAtStart1 ) ? 0 : lNumPts1-1;

      if ( eCrossDir1 == SM_SP_U ) { SER( pSurf1->GetControlPoint( eForm1, lIdx1, ii, sP2, dW2 )); }
      else                         { SER( pSurf1->GetControlPoint( eForm1, ii, lIdx1, sP2, dW2 )); }

      // Now the neighboring points.  For G1, the three points are collinear.
      // That means that sP2 is on the line connecting sP1 and sP3.
      // To do that, we find the point on that line where we want sP2 to be,
      // and then either move sP2 to that point, or move sP1 and sP3 by the
      // same amount in the other direction, depending on how much the caller
      // wants to allow the common end point to move.

      lIdx1 = ( bAtStart1 ) ? 1 : lNumPts1-2;
      lIdx2 = ( bAtStart2 ) ? 1 : lNumPts2-2;

      if ( eCrossDir1 == SM_SP_U ) { SER( pSurf1->GetControlPoint( eForm1, lIdx1, ii, sP1, dW1 )); }
      else                         { SER( pSurf1->GetControlPoint( eForm1, ii, lIdx1, sP1, dW1 )); }

      if ( eCrossDir2 == SM_SP_U ) { SER( pSurf2->GetControlPoint( eForm2, lIdx2, ii, sP3, dW3 )); }
      else                         { SER( pSurf2->GetControlPoint( eForm2, ii, lIdx2, sP3, dW3 )); }

      // Find the point on the line from sP1 to sP3.

      // Get the point on the line, using the calculated fraction, and the shift vector.
      SmPoint3d sPtOnLine( sP1 + dFrac * ( sP3 - sP1 ) );
      SmVector3d sShift( sP2 - sPtOnLine );

      sShift *= dPreserveEndFactor;

      sP1 += sShift;
      sP3 += sShift;
      sP2 = sPtOnLine + sShift;

      // Stuff these back into the surfaces.
      if ( eCrossDir1 == SM_SP_U )
        {
          if ( bAtStart1 )
            {
              SER( pSurf1->SetControlPoint( eForm1, 0, ii, sP2, dW2 ));
              SER( pSurf1->SetControlPoint( eForm1, 1, ii, sP1, dW1 ));
            }
          else
            {
              SER( pSurf1->SetControlPoint( eForm1, lNumPts1-1, ii, sP2, dW2 ));
              SER( pSurf1->SetControlPoint( eForm1, lNumPts1-2, ii, sP1, dW1 ));
            }
        }
      else
        {
          if ( bAtStart1 )
            {
              SER( pSurf1->SetControlPoint( eForm1, ii, 0, sP2, dW2 ));
              SER( pSurf1->SetControlPoint( eForm1, ii, 1, sP1, dW1 ));
            }
          else
            {
              SER( pSurf1->SetControlPoint( eForm1, ii, lNumPts1-1, sP2, dW2 ));
              SER( pSurf1->SetControlPoint( eForm1, ii, lNumPts1-2, sP1, dW1 ));
            }
        }
      if ( eCrossDir2 == SM_SP_U )
        {
          if ( bAtStart2 )
            {
              SER( pSurf2->SetControlPoint( eForm2, 0, ii, sP2, dW2 ));
              SER( pSurf2->SetControlPoint( eForm2, 1, ii, sP3, dW3 ));
            }
          else
            {
              SER( pSurf2->SetControlPoint( eForm2, lNumPts2-1, ii, sP2, dW2 ));
              SER( pSurf2->SetControlPoint( eForm2, lNumPts2-2, ii, sP3, dW3 ));
            }
        }
      else
        {
          if ( bAtStart2 )
            {
              SER( pSurf2->SetControlPoint( eForm2, ii, 0, sP2, dW2 ));
              SER( pSurf2->SetControlPoint( eForm2, ii, 1, sP3, dW3 ));
            }
          else
            {
              SER( pSurf2->SetControlPoint( eForm2, ii, lNumPts2-1, sP2, dW2 ));
              SER( pSurf2->SetControlPoint( eForm2, ii, lNumPts2-2, sP3, dW3 ));
            }
        }

    } // end loop adjusting all control points along the seam.

  return SM_SUCCESS;

} // end SmBSplineSurface::SmoothJoin

