// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*___*/
/**********************************************************************
* FILE NAME --- smsurf_test.cpp
* PURPOSE --- Test file for surface methods.
*
**********************************************************************/
/*___*/

#include <StdAfx.h>
#include <SmSmlibAll.h>
#include <time.h>
#include <prog_test.h>
#include <smCurveTest.h>
#include <smSurfaceTest.h>
#include <SmAssertArray.h>
#include <SmParabola.h>
#include <SmSTEPSurface.h>

#include <nurbs.h>

/***********************************************************************
PURPOSE ---

USAGE NOTES ---
***********************************************************************/
static void my_load_surf1_pts(SmTArray<SmPoint3d> & rPnts)
{
    rPnts.Add(SmPoint3d(0,0,-0.6));
    rPnts.Add(SmPoint3d(1,0.1,0.4));
    rPnts.Add(SmPoint3d(2,0.2,0.3));
    rPnts.Add(SmPoint3d(3,0.3,0));

    rPnts.Add(SmPoint3d(0,1,0.2));
    rPnts.Add(SmPoint3d(1,1,1.5));
    rPnts.Add(SmPoint3d(2,1,1.6));
    rPnts.Add(SmPoint3d(3,1,0.2));

    rPnts.Add(SmPoint3d(0,2,0.3));
    rPnts.Add(SmPoint3d(1,2,1.6));
    rPnts.Add(SmPoint3d(2,2,1.5));
    rPnts.Add(SmPoint3d(3,2,0.4));

    rPnts.Add(SmPoint3d(0,3,0.4));
    rPnts.Add(SmPoint3d(1,3,1.2));
    rPnts.Add(SmPoint3d(2,3,1.1));
    rPnts.Add(SmPoint3d(3,3,0.7));

    rPnts.Add(SmPoint3d(0,4,0.4));
    rPnts.Add(SmPoint3d(1,4,-1.2));
    rPnts.Add(SmPoint3d(2,4,-1.1));
    rPnts.Add(SmPoint3d(3,4,0.5));

    rPnts.Add(SmPoint3d(0,5,0.4));
    rPnts.Add(SmPoint3d(1,5,0.2));
    rPnts.Add(SmPoint3d(2,5,0.1));
    rPnts.Add(SmPoint3d(3,5,0.5));

    return;

} // end my_load_surf1_pts

static void my_load_surf2_pts(SmTArray<SmPoint3d> & rPnts)
{
    rPnts.Add(SmPoint3d(0,0,   3.6));
    rPnts.Add(SmPoint3d(1,0.1, 4.4));
    rPnts.Add(SmPoint3d(2,0.2, 4.3));
    rPnts.Add(SmPoint3d(3,0.3, 4));

    rPnts.Add(SmPoint3d(0,1, 4.2));
    rPnts.Add(SmPoint3d(1,1, 5.5));
    rPnts.Add(SmPoint3d(2,1, 5.6));
    rPnts.Add(SmPoint3d(3,1, 4.2));

    rPnts.Add(SmPoint3d(0,2, 4.3));
    rPnts.Add(SmPoint3d(1,2, 5.6));
    rPnts.Add(SmPoint3d(2,2, 5.5));
    rPnts.Add(SmPoint3d(3,2, 4.4));

    rPnts.Add(SmPoint3d(0,3, 4.4));
    rPnts.Add(SmPoint3d(1,3, 5.2));
    rPnts.Add(SmPoint3d(2,3, 5.1));
    rPnts.Add(SmPoint3d(3,3, 4.7));
    return;
}

static void my_load_surf3_pts(SmTArray<SmPoint3d> & rPnts)
{
    rPnts.Add(SmPoint3d(0,0,-0.6));
    rPnts.Add(SmPoint3d(1,0.1,0.4));
    rPnts.Add(SmPoint3d(2,0.2,0.3));
    rPnts.Add(SmPoint3d(3,0.3,0));
    rPnts.Add(SmPoint3d(3.3,0.0,-0.3));
    rPnts.Add(SmPoint3d(3.7,0.0,-0.7));
    rPnts.Add(SmPoint3d(4,0.0,-1));

    rPnts.Add(SmPoint3d(0,1,0.2));
    rPnts.Add(SmPoint3d(1,1,1.5));
    rPnts.Add(SmPoint3d(2,1,1.6));
    rPnts.Add(SmPoint3d(3,1,0.2));
    rPnts.Add(SmPoint3d(3.3,1,-0.3));
    rPnts.Add(SmPoint3d(3.7,1,-0.7));
    rPnts.Add(SmPoint3d(4,1,-1));

    rPnts.Add(SmPoint3d(0,2,0.3));
    rPnts.Add(SmPoint3d(1,2,1.6));
    rPnts.Add(SmPoint3d(2,2,1.5));
    rPnts.Add(SmPoint3d(3,2,0.4));
    rPnts.Add(SmPoint3d(3.3,2,-0.3));
    rPnts.Add(SmPoint3d(3.7,2,-0.7));
    rPnts.Add(SmPoint3d(4,2,-1));

    rPnts.Add(SmPoint3d(0,3,0.4));
    rPnts.Add(SmPoint3d(1,3,1.2));
    rPnts.Add(SmPoint3d(2,3,1.1));
    rPnts.Add(SmPoint3d(3,3,0.7));
    rPnts.Add(SmPoint3d(3.3,3,-0.3));
    rPnts.Add(SmPoint3d(3.7,3,-0.7));
    rPnts.Add(SmPoint3d(4,3,-1));

    rPnts.Add(SmPoint3d(0,4,0.4));
    rPnts.Add(SmPoint3d(1,4,-1.2));
    rPnts.Add(SmPoint3d(2,4,-1.1));
    rPnts.Add(SmPoint3d(3,4,0.5));
    rPnts.Add(SmPoint3d(3.3,4,-0.3));
    rPnts.Add(SmPoint3d(3.7,4,-0.7));
    rPnts.Add(SmPoint3d(4,4,-1));

    rPnts.Add(SmPoint3d(0,5,0.4));
    rPnts.Add(SmPoint3d(1,5,0.2));
    rPnts.Add(SmPoint3d(2,5,0.1));
    rPnts.Add(SmPoint3d(3,5,0.5));
    rPnts.Add(SmPoint3d(3.3,5,-0.3));
    rPnts.Add(SmPoint3d(3.7,5,-0.7));
    rPnts.Add(SmPoint3d(4,5,-1));

    rPnts.Add(SmPoint3d(0,5.3,-0.3));
    rPnts.Add(SmPoint3d(1,5.3,-0.3));
    rPnts.Add(SmPoint3d(2,5.3,-0.3));
    rPnts.Add(SmPoint3d(3,5.3,-0.3));
    rPnts.Add(SmPoint3d(3.3,5.3,-0.5));
    rPnts.Add(SmPoint3d(3.7,5.3,-0.7));
    rPnts.Add(SmPoint3d(4,5.3,-1));

    rPnts.Add(SmPoint3d(0,5.7,-0.7));
    rPnts.Add(SmPoint3d(1,5.7,-0.7));
    rPnts.Add(SmPoint3d(2,5.7,-0.7));
    rPnts.Add(SmPoint3d(3,5.7,-0.7));
    rPnts.Add(SmPoint3d(3.3,5.7,-0.8));
    rPnts.Add(SmPoint3d(3.7,5.7,-0.9));
    rPnts.Add(SmPoint3d(4,5.7,-1.0));

    rPnts.Add(SmPoint3d(0,6,-1));
    rPnts.Add(SmPoint3d(1,6,-1));
    rPnts.Add(SmPoint3d(2,6,-1));
    rPnts.Add(SmPoint3d(3,6,-1));
    rPnts.Add(SmPoint3d(3.3,6,-1));
    rPnts.Add(SmPoint3d(3.7,6,-1));
    rPnts.Add(SmPoint3d(4,6,-1));
    return;

} // end my_load_surf3_pts

static void my_load_surf4_pts(SmTArray<SmPoint3d> & rPnts)
{
    rPnts.Add(SmPoint3d(0,0,  0.6));
    rPnts.Add(SmPoint3d(1,0.1,-0.4));
    rPnts.Add(SmPoint3d(2,0.2,-0.3));
    rPnts.Add(SmPoint3d(3,0.3,-0));

    rPnts.Add(SmPoint3d(0,1,-0.2));
    rPnts.Add(SmPoint3d(1,1,-1.5));
    rPnts.Add(SmPoint3d(2,1,-1.6));
    rPnts.Add(SmPoint3d(3,1,-0.2));

    rPnts.Add(SmPoint3d(0,2,-0.3));
    rPnts.Add(SmPoint3d(1,2,-1.6));
    rPnts.Add(SmPoint3d(2,2,-1.5));
    rPnts.Add(SmPoint3d(3,2,-0.4));

    rPnts.Add(SmPoint3d(0,3,-0.4));
    rPnts.Add(SmPoint3d(1,3,-1.2));
    rPnts.Add(SmPoint3d(2,3,-1.1));
    rPnts.Add(SmPoint3d(3,3,-0.7));

    rPnts.Add(SmPoint3d(0,4,-0.4));
    rPnts.Add(SmPoint3d(1,4, 1.2));
    rPnts.Add(SmPoint3d(2,4, 1.1));
    rPnts.Add(SmPoint3d(3,4,-0.5));

    rPnts.Add(SmPoint3d(0,5,-0.4));
    rPnts.Add(SmPoint3d(1,5,-0.2));
    rPnts.Add(SmPoint3d(2,5,-0.1));
    rPnts.Add(SmPoint3d(3,5,-0.5));

    return;

} // end my_load_surf4_pts

/***********************************************************************
PURPOSE ---

USAGE NOTES ---
***********************************************************************/
SmBSplineCurve* my_create_3dcrv
 (SmContext       & crContext,
  const SmPoint3d & crTranslate,
  double            dRed,
  double            dGreen,
  double            dBlue)
{
    SmTArray<SmPoint3d> sPnts1;
    sPnts1.Add(SmPoint3d(0,0.0,-1.0));
    sPnts1.Add(SmPoint3d(0,0.5,2.0));
    sPnts1.Add(SmPoint3d(0,1,-1));
    sPnts1.Add(SmPoint3d(0,1.5,0));
    sPnts1.Add(SmPoint3d(0,2,2));
    sPnts1.Add(SmPoint3d(0,2.5,-1.2));
    sPnts1.Add(SmPoint3d(0,3.0,0));
    sPnts1.Add(SmPoint3d(0,3.5,2.0));
    sPnts1.Add(SmPoint3d(0,4,0));
    sPnts1.Add(SmPoint3d(0,4.5,-2.4));
    sPnts1.Add(SmPoint3d(0,5,0.0));
    sPnts1.Add(SmPoint3d(0,5.5,1));
    sPnts1.Add(SmPoint3d(0,6.0,-1));

    SmBSplineCurve *pNurb1 = my_create_nurb(crContext,crTranslate,sPnts1,3,dRed,dGreen,dBlue);
    return pNurb1;

} // end my_create_3dcrv
static SmBoolean sbCreases = FALSE;

/***********************************************************************
PURPOSE ---

USAGE NOTES ---
***********************************************************************/
SmBSplineSurface * my_create_crease_surf
 (SmContext & crContext,
  const SmVector3d & rTranslate,
  SmBoolean bAddCreases = FALSE,
  double dRed=0, double dGreen=0, double dBlue=0)
{
    SmTArray<SmPoint3d> sPnts;
    my_load_surf3_pts(sPnts);

    ULONG lNumU = 9;
    ULONG lNumV = 7;
    ULONG lUDegree = 3;
    ULONG lVDegree = 3;

    ULONG lNumUKnot = lNumU - lUDegree + 1;
    if ( bAddCreases )
      { lNumUKnot -= 2; }
    SmTArray<ULONG> sUKnotMult(lNumUKnot);
    SmTArray<double> sUKnots(lNumUKnot);
    sUKnotMult.Add(lUDegree+1);
    sUKnots.Add(0.0);
    for (ULONG i=1; i<lNumUKnot-1; i++) {
        sUKnots.Add( ((double)i)/(lNumUKnot-1) );
        sUKnotMult.Add(1);
    }
    sUKnotMult.Add(lUDegree+1);
    if ( bAddCreases )
      { sUKnotMult[lNumUKnot-2] = lUDegree; }
    sUKnots.Add(1.0);

    ULONG lNumVKnot = lNumV - lVDegree + 1;
    if ( bAddCreases )
      { lNumVKnot -= 2; }
    SmTArray<ULONG> sVKnotMult(lNumVKnot);
    SmTArray<double> sVKnots(lNumVKnot);
    sVKnotMult.Add(lVDegree+1);
    sVKnots.Add(0.0);
    for (ULONG ii=1; ii<lNumVKnot-1; ii++) {
        sVKnots.Add( ((double)ii)/(lNumVKnot-1) );
        sVKnotMult.Add(1);
    }
    sVKnotMult.Add(lVDegree+1);
    if ( bAddCreases )
      { sVKnotMult[lNumVKnot-2] = lVDegree; }
    sVKnots.Add(1.0);

    SmBSplineSurface *pNurb = NULL ;
    SmTArray<SmPoint3d> sPoints;
    sPoints.SetSize(sPnts.GetSize());
    for (ULONG j=0; j<sPnts.GetSize(); j++) {
        sPoints[j] = sPnts[j] + rTranslate;
    }
    SE(SmBSplineSurface::CreateCanonical(crContext,lUDegree,lVDegree,sPoints,SM_SF_UNSPECIFIED,
        sUKnotMult,sVKnotMult,sUKnots,sVKnots,SM_KT_UNSPECIFIED,NULL,NULL,pNurb));
#ifdef SM_GFX_CODE
    if (smGet_DoGraphics()) {
        smgfx_Erase() ;
        smgfx_SetLook(1,2, dRed,dGreen,dBlue); pNurb->DrawUV(); sm_GraphicsLoop() ;
        sm_GraphicsLoop() ;
    }
#else
  SM_REF3(dBlue, dGreen, dRed);
#endif // SM_GFX_CODE
    return pNurb;

} // end my_create_crease_surf


/***********************************************************************
PURPOSE ---

USAGE NOTES ---
***********************************************************************/
SmBSplineSurface * my_create_nurb_surf
 (const SmContext & crContext,
  const SmPoint3d & rTranslate,
  SmTArray<SmPoint3d> & rPoints,
  ULONG lNumU,
  ULONG lNumV,
  ULONG lUDegree,
  ULONG lVDegree,
  double dRed,
  double dGreen,
  double dBlue)
{
    ULONG lNumUKnot = lNumU - lUDegree + 1;
    SmTArray<ULONG> sUKnotMult(lNumUKnot);
    SmTArray<double> sUKnots(lNumUKnot);
    sUKnotMult.Add(lUDegree+1);
    sUKnots.Add(0.0);
    for (ULONG i=1; i<lNumUKnot-1; i++) {
        sUKnots.Add( ((double)i)/(lNumUKnot-1) );
        sUKnotMult.Add(1);
    }
    sUKnotMult.Add(lUDegree+1);
    sUKnots.Add(1.0);

    ULONG lNumVKnot = lNumV - lVDegree + 1;
    SmTArray<ULONG> sVKnotMult(lNumVKnot);
    SmTArray<double> sVKnots(lNumVKnot);
    sVKnotMult.Add(lVDegree+1);
    sVKnots.Add(0.0);
    for (ULONG ii=1; ii<lNumVKnot-1; ii++) {
        sVKnots.Add( ((double)ii)/(lNumVKnot-1) );
        sVKnotMult.Add(1);
    }
    sVKnotMult.Add(lVDegree+1);
    sVKnots.Add(1.0);



    SmBSplineSurface *pNurb = NULL ;
    SmTArray<SmPoint3d> sPoints;
    sPoints.SetSize(rPoints.GetSize());
    for (ULONG j=0; j<rPoints.GetSize(); j++) {
        sPoints[j] = rPoints[j] + rTranslate;
    }
    SmBSplineSurface::CreateCanonical(crContext,lUDegree,lVDegree,sPoints,SM_SF_UNSPECIFIED,
        sUKnotMult,sVKnotMult,sUKnots,sVKnots,SM_KT_UNSPECIFIED,NULL,NULL,pNurb);
#ifdef SM_GFX_CODE
    if (smGet_DoGraphics()) {
        smgfx_SetColor(dRed,dGreen,dBlue);
        pNurb->Draw();
    }
#else
  SM_REF3(dRed, dGreen, dBlue);
#endif // SM_GFX_CODE
    return pNurb;

} // end my_create_nurb_surf

/***********************************************************************
PURPOSE ---

USAGE NOTES ---
***********************************************************************/
SmBSplineSurface * my_create_plane
 (const SmContext & crContext,
  const SmPoint3d & crTranslate,
  const SmPoint3d & crOrigin,
  const SmPoint3d & crU0Pnt,
  const SmPoint3d & crV0Pnt,
  double dRed,
  double dGreen,
  double dBlue)
{
    SmTArray<SmPoint3d> sPnts;
    sPnts.Add(crOrigin);
    sPnts.Add(crU0Pnt);
    sPnts.Add(crV0Pnt);
    sPnts.Add(crU0Pnt + (crV0Pnt - crOrigin));
    return my_create_nurb_surf(crContext, crTranslate, sPnts, 2, 2, 1, 1, dRed,dGreen,dBlue);

} // end my_create_plane


/***********************************************************************
PURPOSE --- Call SmSurface::GlobalPointSolve on given surface
            over an array of evenly spaced sample points taken
            from the surface's bounding box.

USAGE NOTES --- When eSolverOperation == SM_SO_RAYFIRE
            call SmSurface::GlobalLineIntersect instead.
***********************************************************************/
SmStatus my_test_surface_point_extrema
  (const SmBSplineSurface & crSurface,          // in : target surface
   const SmExtent2d       & crUVDomain,         // in : target curve
   SmSolverOperationType    eSolverOperation,   // in : operation to exercise
   long                     lNumX,              // in : number of evenly spaced X sample locations
   long                     lNumY,              // in : number of evenly spaced Y sample locations
   long                     lNumZ,              // in : number of evenly spaced Y sample locations
   ULONG                  & rlNumFound)         // out: total number of solutions found
{
  // init output
  rlNumFound = 0;

  // locals
  SmExtent3d sBBox;
  ULONG lCount=0;
  SmVector3d sLineVec(0.0,0.0,1.0);
  SmSolution aSols[10];
  SmSolutionArray sSolutions(10,aSols);

  // get surface boundingBox and a size scale
  SER(crSurface.CalculateBoundingBox(crUVDomain,&sBBox,NULL));
  // double dFindDistance = (sBBox.GetMax().DistanceBetween(sBBox.GetMin()))/5.0;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if(bDebugMe)
    {
      crSurface.Dump() ;
      sBBox.Dump() ;
    }
#ifdef SM_GFX_CODE
  if (smGet_DoGraphics()) {
      smgfx_Erase();
      smgfx_SetLook(1, 2, 0, 1, 1); crSurface.DrawUV(); sm_GraphicsLoop();
      smgfx_SetLook(1, 2, 1, 1, 0); sBBox.Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop();
  }
#endif // SM_GFX_CODE
#endif // SM_DEBUG_CODE

  // for every X sample point
  for (long i=0; i<lNumX; i++)
    {
      // set x sample point distance from Surface BoundingBox min corner
      double dX = (sBBox.GetMax().x - sBBox.GetMin().x) / (lNumX-1+SM_EFF_ZERO);
      if (dX < SM_EFF_ZERO) i  = lNumX;
      if (lNumX < 2)        dX = 0.0;

      // for every Y sample point
      for (long j=0; j<lNumY; j++)
        {
          // set y sample point distance from Surface BoundingBox min corner
          double dY = (sBBox.GetMax().y - sBBox.GetMin().y) / (lNumY-1+SM_EFF_ZERO);
          if (dY < SM_EFF_ZERO) j  = lNumY;
          if (lNumY < 2)        dY = 0.0;

          // for every Z sample point
          for (long k=0; k<lNumZ; k++)
            {
              // set z sample point distance from Surface BoundingBox min corner
              double dZ = (sBBox.GetMax().z - sBBox.GetMin().z) / (lNumZ-1+SM_EFF_ZERO);
              if (dZ < SM_EFF_ZERO) k  = lNumZ;
              if (lNumZ < 2)        dZ = 0.0;

              // get sample point
              SmPoint3d sTgtPoint( sBBox.GetMin().x + i * dX,
                                   sBBox.GetMin().y + j * dY,
                                   sBBox.GetMin().z + k * dZ);

              // Get Surface/Point solution for this sample point
              if (eSolverOperation == SM_SO_RAYFIRE)
                {
                  k = lNumZ;
                  sTgtPoint = sTgtPoint - sLineVec*10.0;
                  SER(crSurface.GlobalLineIntersect(crUVDomain,    // in : Surface domain to intersect
                                                    sTgtPoint,         // in : Point on the infinite line
                                                    sLineVec,      // in : Direction vector of the infinite line
                                                    NULL,          // in : If specified bounds the line to a specific segment
                                                    TRUE,          // in : If TRUE specifies that the line is bounded only at the
                                                                   //      start point and proceeds along the vector to infinity.
                                                    SM_ZONE_TOL_3D,        // in : min distance between distinct 3d points
                                                    sSolutions));  // out: Solution array
                }
              else
                {
                  // make the Surface/Point solve call
                  SER(crSurface.GlobalPointSolve
                       (crUVDomain,       // in : Domain of surface to search for solutions
                        eSolverOperation, // in : oneof: SM_SO_MINIMIZE, SM_SO_MAXIMIZE, SM_SO_NORMALIZE, SM_SO_INTERSECT
                        sTgtPoint,            // in : Target point for the solve operation
                        SM_ZONE_TOL_3D,           // in :
                        NULL,             // in : Max Drop distance for min/max and normalize operations.
                                          //      NULL to ignore.
                        SM_SR_ALL,        // in : SM_SR_SINGLE=get best solution, SM_SR_ALL=get all solutions
                        sSolutions));     // out: array of problem solutions reported as surface UV parameter values
                }

              // Check that minimization and maximization always find answers
              if (   eSolverOperation == SM_SO_MINIMIZE
                  || eSolverOperation == SM_SO_MAXIMIZE)
                {
                  SM_ASSERT(sSolutions.GetSize() > 0);
                  if (sSolutions.GetSize() < 1) // error branch - expect some solutions
                    {
#ifdef SM_GFX_CODE
                      if (smGet_DoGraphics()) {
                          SmSurfaceCache *pCache = smsurf_GetSurfaceCache(&crSurface);
                          crSurface.Dump();
                          if (pCache) pCache->Dump();

                          smgfx_Erase();
                          smgfx_SetLook(1, 2, 0, 1, 1); crSurface.DrawUV(); sm_GraphicsLoop();
                          smgfx_SetLook(1, 2, 0, 1, 0); if (pCache) pCache->Draw(); sm_GraphicsLoop();
                          smgfx_SetLook(1, 2, 1, 1, 0); sBBox.Draw(); sm_GraphicsLoop();
                          smgfx_SetLook(3, 4, 1, 0, 0); sTgtPoint.Draw(); sm_GraphicsLoop();
                          smgfx_SetLook(4, 5, 1, 0, 1); sSolutions.Draw(); sm_GraphicsLoop();
                          sm_GraphicsLoop();
                      }
#endif // SM_GFX_CODE
                    }
                } // end SolverOp == MIN or MAX operation

              // count number of SurfacePoint Solves
              lCount++;

              // for every solution
              for (ULONG kk=0; kk<sSolutions.GetSize(); kk++)
                {
                  // count number of found solutions
                  rlNumFound ++;

                  // when doing graphics
                  if (smGet_DoGraphics())
                    {
                      SmSolution sSolution = sSolutions[kk];
                      SmPoint3d sFndPnt;
                      if (eSolverOperation == SM_SO_RAYFIRE)
                        {
//                            SmPoint2d sUV(sSolution.m_vStart[1],sSolution.m_vStart[2]);
//                            SER(crSurface.EvaluatePoint(sUV,sFndPnt));
//                            SmVector3d sTo = sFndPnt - sTgtPoint;
//                            sTo.Draw(&sTgtPoint);
//                            sTgtPoint.Draw();
                        }
                      else
                        {
                          SmPoint2d sUV(sSolution.m_vStart[0],sSolution.m_vStart[1]);
                          SER(crSurface.EvaluatePoint(sUV,sFndPnt));

#ifdef SM_GFX_CODE
SmBoolean bRefresh = FALSE;
                          if ( bRefresh ) {
                              smgfx_Erase() ;
                              smgfx_SetLook(1,2, 0,1,1); crSurface.DrawUV() ; sm_GraphicsLoop();
                          }
                          smgfx_SetLook(2,3, 1,0,0); sFndPnt.DrawPointToPoint(sTgtPoint) ; sm_GraphicsLoop();
                          sm_GraphicsLoop();
#endif // SM_GFX_CODE
                        } // end rayfire/other operation branch
                    } // end smGet_DoGraphics() check
                } // end iter every Surface/Point solution

#ifdef SM_GFX_CODE
              if (smGet_DoGraphics()) {
                  // Draw blue point if no solutions.
                  if (sSolutions.GetSize() < 1) {
                      smgfx_SetLook(3, 5, 0, 0, 1); sTgtPoint.Draw(); sm_GraphicsLoop();
                      sm_GraphicsLoop();
                  }
              }
#endif // SM_GFX_CODE
            } // end iter every Z sample increment
        } // end iter every Y sample increment
    } // end iter every X sample increment

  return SM_SUCCESS;


} // end my_test_surface_point_extrema

/***********************************************************************
PURPOSE ---

USAGE NOTES ---
***********************************************************************/
SmStatus my_test_surface_point_intersect
 (const SmBSplineSurface & crSurface,
  const SmExtent2d       & crUVDomain,
  long                     lNumU,
  long                     lNumV)
{
    SmSolutionArray sSolutions;
    ULONG lCount=0;

    for (long i=0; i<lNumU; i++) {
        for (long j=0; j<lNumV; j++) {
            SmPoint2d sUV = crUVDomain.Evaluate( i/(lNumU-1.0), j/(lNumV-1.0) );
            SmPoint3d sEvalPnt;
            crSurface.EvaluatePoint(sUV,sEvalPnt);
            SER(crSurface.GlobalPointSolve(crUVDomain,
                SM_SO_INTERSECT,sEvalPnt,SM_ZONE_TOL_3D,NULL,SM_SR_ALL,sSolutions));
            lCount++;
            if (sSolutions.GetSize() == 0) { // error branch - expect one solution per iteration
#ifdef SM_GFX_CODE
                if (smGet_DoGraphics()) {
                    smgfx_Erase();
                    smgfx_SetLook(1, 1, 0, 1, 1); crSurface.DrawUV(); sm_GraphicsLoop();
                    smgfx_SetLook(1, 4, 1, 0, 0); sEvalPnt.Draw(); sm_GraphicsLoop();
                    sm_GraphicsLoop();
                    ERR(SM_ERR);
                    if (FALSE) {
                        sSolutions.Dump();
                    }
                }
#endif // SM_GFX_CODE
            } // end sSolutions.GetSize() == 0 check - an error, we expect one solution every time
        } // end iter j
    } // end iter i

    return SM_SUCCESS;

} // end my_test_surface_point_intersect

/***********************************************************************
PURPOSE ---

USAGE NOTES ---
***********************************************************************/
SmStatus my_test_surface_closest_point
 (const SmBSplineSurface & crSurface,
  SmPoint3d              & point )
{

    SmExtent2d crUVDomain = crSurface.GetNaturalUVDomain();

    double tolerance = 1.e-006;
    double max_distance=0.03;

    SmSolutionArray sSolutions;
    crSurface.GlobalPointSolve( crUVDomain, SM_SO_MINIMIZE,point, tolerance, &max_distance, SM_SR_SINGLE, sSolutions);

    if( sSolutions.GetSize() <= 0 )
        return SM_ERR;

    SmPoint2d uvOnSrf( sSolutions[0].m_vStart[0], sSolutions[0].m_vStart[1] );

    SmPoint3d ptOnSrf;
    crSurface.EvaluatePoint( uvOnSrf, ptOnSrf );

    double dist = point.DistanceBetween( ptOnSrf );

    if( dist > max_distance )
        return SM_ERR;

    return SM_SUCCESS;

} // end my_test_surface_closest_point

/***********************************************************************
PURPOSE ---

USAGE NOTES ---
***********************************************************************/
SmStatus my_test_planar_section
  (const SmBSplineSurface & crSurface,        // in :
   const SmExtent2d       & crUVDomain,       // in :
   const SmVector3d       & crPlaneNormal,    // in :
   ULONG                    lNumPlanes,       // in :
   ULONG                  & rlNumFound)       // out:
{
  // init output
  rlNumFound = 0;

  // context
  SmContext sContext;
  const SmContext *pContext =   crSurface.GetContext()
                              ? crSurface.GetContext()
                              : &sContext ;

  // locals
  SmExtent3d sBBox;
  SER(crSurface.CalculateBoundingBox(crUVDomain,&sBBox,NULL));

#ifdef SM_GFX_CODE // draw surface
  double dBBoxSize = sBBox.GetSize().Length() ;
  if (smGet_DoGraphics()) {
      smgfx_Erase();
      smgfx_SetLook(1, 2, 0, 1, 1); crSurface.DrawUV(); sm_GraphicsLoop();
      sm_GraphicsLoop();
  }
#endif // SM_GFX_CODE 

  // for every plane
  for (ULONG i=0; i<lNumPlanes; i++)
    {
      // pick a plane
      double dParam = (i*1.0) / (lNumPlanes-1.0);
      SmPoint3d sPlanePoint = sBBox.Evaluate(dParam,dParam,dParam);

      // find planar 3d curves
      SmTArray<SmCurve*> s3DCurves;
      SmTArray<SmCurve*> sUVCurves;
      SER(crSurface.CreatePlanarSectionCurves(*pContext,   crUVDomain,
                                              sPlanePoint, crPlaneNormal,
                                              NULL,        NULL,
                                              &s3DCurves, &sUVCurves));

#ifdef SM_GFX_CODE // draw in section curve
      if (smGet_DoGraphics()) {

          if (s3DCurves.GetSize() == 0)
          {
              SmPlane sPlane(sPlanePoint, crPlaneNormal, pContext);
              SmExtent2d sSTEPUVDomain(-dBBoxSize, -dBBoxSize, dBBoxSize, dBBoxSize);
              sPlane.AdjustSTEPUVDomain(sSTEPUVDomain);

              smgfx_SetLook(1, 2, 0, 1, 0); sPlane.DrawUV(); ; sm_GraphicsLoop();
              sm_GraphicsLoop();
          }
      }
#endif // SM_GFX_CODE 

      // for every section curve
      for (ULONG j=0; j<s3DCurves.GetSize(); j++)
        {
          rlNumFound ++;
#ifdef SM_GFX_CODE // draw in section curve
          SmCurve *pCurve = s3DCurves[j];
          if (smGet_DoGraphics()) {
              smgfx_SetLook(2, 4, 1, 0, 0); (SM_CAST_PTR(SmBSplineCurve, pCurve))->Draw(); ; sm_GraphicsLoop();
              sm_GraphicsLoop();

#ifdef SM_DEBUG_CODE
              SmBoolean bDebugMe = FALSE;
              if (bDebugMe)
              {
                  pCurve->Dump();
              }
#endif // SM_DEBUG_CODE
          }
#endif // SM_GFX_CODE 

          SM_ASSERT(s3DCurves[j] != NULL) ; delete s3DCurves[j] ; s3DCurves[j] = NULL ;
          SM_ASSERT(sUVCurves[j] != NULL) ; delete sUVCurves[j] ; sUVCurves[j] = NULL ;
        }
    } // end iter every plane

  // all done
  return SM_SUCCESS;

} // end my_test_planar_section

/***********************************************************************
PURPOSE ---

USAGE NOTES ---
***********************************************************************/
SmStatus my_test_parallel_projection
 (const SmBSplineSurface & crSurface,
  const SmExtent2d       & crUVDomain,
  ULONG                    lNumCurves,
  ULONG                  & rlNumFound)
{
    rlNumFound = 0;
    SmExtent3d sBBox;

    SmContext sContext;
    const SmContext *pContext =   crSurface.GetContext()
                                ? crSurface.GetContext()
                                : &sContext ;

    SER(crSurface.CalculateBoundingBox(crUVDomain,&sBBox,NULL));
    SmPoint3d sCent = sBBox.Evaluate(0.5,0.5,1.0);
    SmVector3d sDir = sBBox.Evaluate(0.5,0.5,0.0) - sCent;
    SmVector3d sWidVec = sBBox.Evaluate(1,1,1) - sBBox.Evaluate(0,0,1);
    double dMaxRadius = smos_Max(sWidVec.x,sWidVec.y);

    for (ULONG i=1; i<=lNumCurves; i++) {
        double dCurrRad = (i*1.0)/(lNumCurves) * (dMaxRadius/2.0);
        SmBSplineCurve * pCurve = my_create_circle(*pContext,dCurrRad,sCent,SM_CO_QUADRATIC,0,0,1);
        NER(pCurve);
        SmObjDelete sCurveCleanup(pCurve);
        SmTArray<SmCurve*> s3DCurves;
        SmTArray<SmCurve*> sUVCurves;
        SER(crSurface.CreateParallelProjectionCurves(*pContext,crUVDomain,*pCurve,
            SmVector3d(0,0,-1),NULL,NULL,NULL,&s3DCurves,&sUVCurves));
        for (ULONG j=0; j<s3DCurves.GetSize(); j++) {
            rlNumFound ++;
#ifdef SM_GFX_CODE
            SmCurve *pCrv = s3DCurves[j];
            if (smGet_DoGraphics()) {
                smgfx_SetColor(1, 0, 0);
                pCrv->DrawWDeriv(pCrv->GetNaturalInterval(), 0);
                if (FALSE) {
                    sUVCurves[j]->DrawWDeriv(sUVCurves[j]->GetNaturalInterval(), 0);
                }
            }
#endif // SM_GFX_CODE
            SM_ASSERT(s3DCurves[j] != NULL) ; delete s3DCurves[j] ; s3DCurves[j] = NULL ;
            SM_ASSERT(sUVCurves[j] != NULL) ; delete sUVCurves[j] ; sUVCurves[j] = NULL ;
        }
    }

    return SM_SUCCESS;

} // end my_test_parallel_projection


/**************************************************************
PURPOSE ---  a surface intersection test case

USAGE NOTES ---  the test: intersect input surface with a sequence of lines
**************************************************************/
SmStatus my_test_surface_curve_intersect
  (const SmBSplineSurface & crSurface,    // in : input target surface
   const SmExtent2d       & crUVDomain,   // in : intersection test range
   long                     lNumX,        // in : number of x direction test iterations
   long                     lNumY,        // in : number of y direction test iterations
   ULONG                  & rlNumFound)   // out: numer of intersections found
{
  // init output
  rlNumFound = 0;

  // locals
  SmExtent3d sBBox;
  SmSolutionArray sSolutions2;
  SmSolutionArray sSolutions;
  ULONG lCount=0;
  SmContext sContext;
  const SmContext *pContext =   crSurface.GetContext()
                              ? crSurface.GetContext()
                              : &sContext ;

#ifdef SM_GFX_CODE
  if (smGet_DoGraphics()) {
      smgfx_Erase();
      smgfx_SetLook(1, 2, 0, 1, 1); crSurface.DrawUV(); sm_GraphicsLoop();
      sm_GraphicsLoop();
  }
#endif // SM_GFX_CODE

  // surface BBox
  SER(crSurface.CalculateBoundingBox(crUVDomain,&sBBox,NULL));
  // double dFindDistance = (sBBox.GetMax().DistanceBetween(sBBox.GetMin()))/5.0;

  // for every Numx division - building lines to intersect with surface
  for (long i=0; i<lNumX; i++)
    {
      double dX = (sBBox.GetMax().x - sBBox.GetMin().x) / (lNumX-1+SM_EFF_ZERO);
      if (dX < SM_EFF_ZERO) i=lNumX;
      if (lNumX < 2) dX = 0.0;

      // for every NumY division
      for (long j=0; j<lNumY; j++)
        {
          double dY = (sBBox.GetMax().y - sBBox.GetMin().y) / (lNumY-1+SM_EFF_ZERO);
          SmPoint3d sStartPt( sBBox.GetMin().x + i * dX,
                              sBBox.GetMin().y + j * dY,
                              sBBox.GetMin().z);
          SmPoint3d sEndPt( sBBox.GetMin().x + i * dX,
                            sBBox.GetMin().y + j * dY,
                            sBBox.GetMax().z);
          if (sBBox.GetMax().z - sBBox.GetMin().z < SM_EFF_ZERO)
            {
              sStartPt.z -= 1.0;
              sEndPt.z   += 1.0;
            }

          SmCurve *pLine = my_create_line(*pContext,sStartPt,sEndPt,0.2,0,0.3);
          SmObjDelete sCleanup(pLine);

          // intersect line with surface
          SER(crSurface.GlobalCurveIntersect(crUVDomain,
                                             *pLine,
                                             pLine->GetNaturalInterval(),
                                             SM_ZONE_TOL_3D,
                                             sSolutions));
          lCount++;

            {
//                SER(crSurface.GlobalLineIntersect(crUVDomain,sStartPt,
//                    SmVector3d(0,0,1),NULL,TRUE,SM_ZONE_TOL_3D,sSolutions));
//                SM_ASSERT(sSolutions2.GetSize() == sSolutions.GetSize());
            }
          rlNumFound += sSolutions.GetSize();

#ifdef SM_GFX_CODE
          if (smGet_DoGraphics()) {
              smgfx_SetLook(1, 2, 1, 0, 1); pLine->Draw(); sm_GraphicsLoop();
              smgfx_SetLook(1, 4, 1, 0, 0); sSolutions.Draw(); sm_GraphicsLoop();
              sm_GraphicsLoop();
          }
#endif // SM_GFX_CODE
          if (FALSE)
            {
              sSolutions.Dump();
            }

        } // end iter j
    } // end iter i - building lines and looking for line/surf intersections

  // all done
  return SM_SUCCESS;

} // end my_test_surface_curve_intersect

/***********************************************************************
PURPOSE ---

USAGE NOTES ---
***********************************************************************/
SmStatus my_test_surface_tessellation
 (const SmBSplineSurface & /* crSur           */,
  const SmExtent2d       & /* crUVDomain      */,
  double                   /* dChordHeightTol */,
  double                   /* dAngleTol       */)
{
    return SM_SUCCESS;

} // end my_test_surface_tessellation

/***********************************************************************
PURPOSE ---

USAGE NOTES ---
***********************************************************************/
static SmStatus my_surface_from_file
 (const SmContext & crContext,
  const TCHAR * pFileName,
  SmBSplineSurface *& rNewSurface)
{
    smos_WriteBuffer(_T("\nReading B-Spline Surface From File: "));
    smos_WriteBuffer(pFileName);
    smos_WriteBuffer(_T("\n\n"));
    SER(SmBSplineSurface::ReadFromFile(crContext,pFileName,rNewSurface));
    return SM_SUCCESS;

} // end my_surface_from_file


/***********************************************************************
PURPOSE ---

USAGE NOTES ---
***********************************************************************/
PT_EXPORT SmStatus my_test_surface_suite(void)
{
    SER(my_test_analytic_surfaces());
    return SM_SUCCESS;

} // end my_test_surface_suite

/***********************************************************************
PURPOSE ---

USAGE NOTES ---
***********************************************************************/
SmStatus my_test_GlobalCSI
  (const SmSurface & crSurface,      // in : target surface
   const SmCurve   & crCurve,        // in : target curve
   double            d3dTol,         // in : intersection tolerance
   ULONG           & rlTsectCount)   // out: number of intersection results
{
#ifdef SM_GFX_CODE
  if (smGet_DoGraphics())
    {
      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1); crSurface.DrawUV() ;
      smgfx_SetLook(3,6, 0,1,0); crCurve.DrawParams() ;
      sm_GraphicsLoop();
    }
#endif // SM_GFX_CODE

  // get surface/curve intersection
  SmSolutionArray sSolutions;
  SER(crSurface.GlobalCurveIntersect(crSurface.GetNaturalUVDomain(),
      crCurve, crCurve.GetNaturalInterval(), d3dTol, sSolutions));

  // set output
  rlTsectCount = sSolutions.GetSize();

#ifdef SM_GFX_CODE
  if (smGet_DoGraphics())
    {
      sSolutions.Dump();

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1); crSurface.DrawUV() ; sm_GraphicsLoop();
      smgfx_SetLook(2,3, 0,1,0); crCurve.DrawParams() ; sm_GraphicsLoop();
      smgfx_SetLook(5,8, 1,0,0); sSolutions.Draw() ; sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_GFX_CODE

  // all done
  return SM_SUCCESS;


} // end my_test_GlobalCSI


/***********************************************************************
PURPOSE ---

USAGE NOTES ---
***********************************************************************/
SmStatus my_test_cs_solve
  (const SmSurface & crSurface,
   const SmCurve & crCurve,
   SmSolverOperationType eSolverOperation,
   const SmVector3d * cpOptVectors,
   ULONG & rlFoundCount)
{
    
    SmSolutionArray sSolutions;
    SER(crSurface.GlobalCurveSolve(crSurface.GetNaturalUVDomain(),crCurve,
        crCurve.GetNaturalInterval(),eSolverOperation,SM_ZONE_TOL_3D,NULL,cpOptVectors,SM_SR_ALL,
        sSolutions));

    rlFoundCount = sSolutions.GetSize();

    if (FALSE) {
        sSolutions.Dump();
    }

    if (smGet_DoGraphics()) {
        for (ULONG kk=0; kk<sSolutions.GetSize(); kk++) {
            SmSolution & rSolution = sSolutions[kk];
            SmPoint3d sFndPnt;
            SmPoint2d sUVFound(rSolution.m_vStart[1],rSolution.m_vStart[2]);
            SER(crSurface.EvaluatePoint(sUVFound,sFndPnt));
#ifdef SM_GFX_CODE
            sFndPnt.Draw();
            SmPoint3d sCrvPnt;
            SER(crCurve.EvaluatePoint(rSolution.m_vStart[0],sCrvPnt));
            if (eSolverOperation == SM_SO_3D_SIGNED_DIRECTED_MINIMIZE) { smgfx_SetColor(0,0,1); }
            else { smgfx_SetColor(1,0,0); }
            sCrvPnt.Draw();
            SmVector3d sVec = sFndPnt - sCrvPnt;
            sVec.Draw(&sCrvPnt);
#endif // SM_GFX_CODE
        }
    }

    return SM_SUCCESS;

} // end my_test_cs_solve

/***********************************************************************
PURPOSE ---

USAGE NOTES ---
***********************************************************************/
SmStatus my_test_ss_solve
 (const SmSurface & crSurface,
  const SmSurface & crSurface2,
  SmSolverOperationType eSolverOperation,
  ULONG & rlFoundCount)
{

    SmSolutionArray sSolutions;
    SER(crSurface.GlobalSurfaceSolve(crSurface.GetNaturalUVDomain(),crSurface2,
    crSurface2.GetNaturalUVDomain(),eSolverOperation,SM_ZONE_TOL_3D,NULL,NULL,SM_SR_ALL,
        sSolutions));

    rlFoundCount = sSolutions.GetSize();

    if (FALSE) {
        sSolutions.Dump();
    }

    if (smGet_DoGraphics()) {
        for (ULONG kk=0; kk<sSolutions.GetSize(); kk++) {
            SmSolution & rSolution = sSolutions[kk];
            SmPoint3d sFndPnt;
            SmPoint2d sUVFound(rSolution.m_vStart[0],rSolution.m_vStart[1]);
            SER(crSurface.EvaluatePoint(sUVFound,sFndPnt));
            sFndPnt.Draw();
            SmPoint3d sFndPnt2;
            SmPoint2d sUVFound2(rSolution.m_vStart[2],rSolution.m_vStart[3]);
            SER(crSurface2.EvaluatePoint(sUVFound2,sFndPnt2));
            sFndPnt2.Draw();
            SmVector3d sVec = sFndPnt - sFndPnt2;
            sVec.Draw(&sFndPnt2);
        }
    }

    return SM_SUCCESS;

} // end my_test_ss_solve

/***********************************************************************
PURPOSE ---

USAGE NOTES ---
***********************************************************************/
PT_EXPORT SmStatus my_test_csi_tangency()
{

//    SmContext sContext(pPool);
    SmContext sContext;

    {
      SmBSplineSurface *pCylBSS = NULL ;
      SmAxis2Placement sA2P;
      sA2P.SetCanonical(SmPoint3d(0,0,0),SmVector3d(1,0,0),SmVector3d(0,1,0));
      SER(SmBSplineSurface::CreateConePatch(sContext,
          sA2P,1.0,1.0,0.0,360.0,1.0,SM_CO_QUINTIC,pCylBSS));
      SmObjDelete sCleanupSur(pCylBSS);
      if (TRUE)
        {
          SER(my_test_surface_tessellation(*pCylBSS,pCylBSS->GetNaturalUVDomain(),0.01,20.0*SM_PI/180.0));
        }


    for (ULONG m=0; m<2; m++)
      {
        double dYDelta = 2.0e-3 / 22.0;
        double dY = 1.0 - 1.0e-3;
        double dYC = -2.0 + 1.0e-3;
        for (ULONG i=0; i<=21; i++)
          {
            SmPoint3d sStartPt( -1.0, dY, i/21.0);
            SmPoint3d sEndPt( 1.0, dY, i/21.0);
            SmPoint3d sCenter( 0.0, dYC, i/21.0);
            dY = dY + dYDelta;
            dYC = dYC - dYDelta;

            SmCurve *pCurve=NULL;
            if (m==1) pCurve = my_create_line(sContext,sStartPt,sEndPt,0,0,0);
            if (m==0) pCurve = my_create_circle(sContext,1.0,sCenter,SM_CO_QUINTIC,0,1,1);
            SmObjDelete sCleanup(pCurve);

            ULONG lTsectCount;
            SER(my_test_GlobalCSI(*pCylBSS,*pCurve,1.0001e-3,lTsectCount));

            // gwc: modified near-tangent intersections to return 2-pt solutions when separated by more than tol
            SM_ASSERT(   (m == 0 && sCenter.y >  -2.0 && lTsectCount == 2)
                      || (m == 0 && sCenter.y <= -2.0 && lTsectCount == 1)
                      || (m == 1 && dY < 1.0  && lTsectCount == 2)
                      || (m == 1 && dY >= 1.0 && lTsectCount == 1)) ;
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
            if(bDebugMe)
              {
                smgfx_Erase() ;
                smgfx_SetLook(1,2, 0,0,1) ; pCurve->Draw() ; sm_GraphicsLoop() ;
                smgfx_SetLook(1,2, 0,1,0) ; pCylBSS->DrawUV() ; sm_GraphicsLoop() ;
                sm_GraphicsLoop() ;
              }
#endif // SM_DEBUG_CODE
          } /* for i */
      } // for m
    }

//    pPool->Free();
//    delete pPool; pPool = NULL ;
    return SM_SUCCESS;

} // end my_test_csi_tangency


/***********************************************************************
PURPOSE ---

USAGE NOTES ---
***********************************************************************/
PT_EXPORT SmStatus my_test_csi_tol(void)
{

//    SmContext sContext(pPool);
  SmContext sContext;

  { // For testing near knots and near end of closed curves
    ULONG lNumFound;

    SmBSplineSurface *pCylBSS1=NULL, *pCylBSS2=NULL;
    SmAxis2Placement sA2P;
    sA2P.SetCanonical(SmPoint3d (0,0,0),
                      SmVector3d(1,0,0),
                      SmVector3d(0,1,0));
    SER(SmBSplineSurface::CreateConePatch(sContext,
                                          sA2P,
                                          1.0,1.0,
                                          0.0,360.0,
                                          1.0,
                                          SM_CO_QUADRATIC,
                                          pCylBSS1));
    SmObjDelete sCU1(pCylBSS1);
#ifdef SM_GFX_CODE
    if (smGet_DoGraphics()) {
        smgfx_SetLook(1, 2, 1, 0, 0); pCylBSS1->Draw(); sm_GraphicsLoop();
        smgfx_SetLook(1, 2, 0, 0, 1); pCylBSS1->DrawUV(1, 1); sm_GraphicsLoop();
        sm_GraphicsLoop();
    }
#endif // SM_GFX_CODE

    sA2P.SetCanonical(SmPoint3d (2.5,0,0),
                      SmVector3d(1,0,0),
                      SmVector3d(0,1,0));
    SER(SmBSplineSurface::CreateConePatch(sContext,
                                          sA2P,
                                          1.0,1.0,
                                          0.0,360.0,
                                          1.0,
                                          SM_CO_QUADRATIC,
                                          pCylBSS2));
    SmObjDelete sCU2(pCylBSS2);
#ifdef SM_GFX_CODE
    if (smGet_DoGraphics()) {
        smgfx_SetLook(1, 2, 1, 0, 0); pCylBSS2->Draw(); sm_GraphicsLoop();
        smgfx_SetLook(1, 2, 0, 1, 0); pCylBSS2->DrawUV(1, 1); sm_GraphicsLoop();
        sm_GraphicsLoop();
    }
#endif // SM_GFX_CODE

    double  y = 0;
    double dy = 1.0001*SM_ZONE_TOL_3D/10.0;
    for (ULONG i=0; i<20; i++ )
      {
        SmBSplineCurve *pLine = my_create_line(sContext,
                                               SmPoint3d(0,  y,0.5+y),
                                               SmPoint3d(2.5,y,0.5+y),
                                               0.5,
                                               0.5,
                                               0.0);
        SmObjDelete sCU3(pLine);

#ifdef SM_GFX_CODE
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
        if(bDebugMe)
          {
            smgfx_Erase() ;
            smgfx_SetLook(1,2, 1,0,0) ; pCylBSS1->Draw(); sm_GraphicsLoop() ;
            smgfx_SetLook(1,2, 0,0,1) ; pCylBSS1->DrawUV(1,1); sm_GraphicsLoop() ;
            smgfx_SetLook(1,2, 1,0,0) ; pCylBSS2->Draw(); sm_GraphicsLoop() ;
            smgfx_SetLook(1,2, 0,1,0) ; pCylBSS2->DrawUV(1,1); sm_GraphicsLoop() ;
            smgfx_SetLook(1,2, 1,0,0) ; pLine->Draw(); sm_GraphicsLoop() ;
            sm_GraphicsLoop() ;
          }
#endif // SM_DEBUG_CODE
#endif // SM_GFX_CODE

        // Near end of cylinder seam - previously the system expected
        //  two solutions near seams when the edge intersected the surface
        //  within tol of the seam, the exact solution and the near-miss seam solution.
        // However, the new thinking is when there is an exact intersection, use that
        //  and have near-miss intersections only when there is a near miss.  The
        //  only intersection that should get two solutions (both sides of the seam)
        //  is the one where the line is within numerical tolerance of the seam.
        SER(my_test_GlobalCSI(*pCylBSS1, *pLine, SM_ZONE_TOL_3D, lNumFound));
        if      (y <= SM_EFF_ZERO) { if (lNumFound != 2)
                                       { SE(SM_ERR); }
                                   }
        else if (y  > SM_EFF_ZERO) { if (lNumFound != 1)
                                       { SE(SM_ERR); }
                                   }
        y += dy;

        // Near knot
        SER(my_test_GlobalCSI(*pCylBSS2, *pLine, SM_ZONE_TOL_3D, lNumFound));
        SM_ASSERT(lNumFound == 1);
      } // end iter 20 tests
  } // end context block

//    pPool->Free();
//    delete pPool; pPool = NULL ;

  return SM_SUCCESS;


} // end my_test_csi_tol

/***********************************************************************
PURPOSE ---

USAGE NOTES ---
***********************************************************************/
PT_EXPORT SmStatus my_test_csi_coincidence()
{
  SmContext sContext;

  // For testing near knots and near end of closed curves
  ULONG lNumFound;

  SmBSplineSurface *pCylBSS1=NULL, *pCylBSS2=NULL;
  SmAxis2Placement sA2P;

  // make SM_CO_QUADRATIC cone patch
  sA2P.SetCanonical(SmPoint3d(0,0,0),SmVector3d(1,0,0),SmVector3d(0,1,0));
  SER(SmBSplineSurface::CreateConePatch(sContext,
      sA2P,1.0,1.0,0.0,360.0,1.0,
      SM_CO_QUADRATIC,pCylBSS1));
  SmObjDelete sCU1(pCylBSS1);

  // make SM_CO_QUINTIC ConePatch
  sA2P.SetCanonical(SmPoint3d(3.0,0,0),SmVector3d(1,0,0),SmVector3d(0,1,0));
  SER(SmBSplineSurface::CreateConePatch(sContext,
      sA2P,1.0,1.0,0.0,360.0,1.0,
      SM_CO_QUINTIC,pCylBSS2));
  SmObjDelete sCU2(pCylBSS2);

#ifdef SM_GFX_CODE
  if (smGet_DoGraphics())
    {
      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1); pCylBSS1->DrawUV() ;
      smgfx_SetLook(1,2, 0,1,0); pCylBSS2->DrawUV() ;
      sm_GraphicsLoop();
    }
#endif // SM_GFX_CODE

  {
    // Test lines which are exactly on top of cylinder
    SmBSplineCurve *pLine = my_create_line(sContext,SmPoint3d(0,1,0),SmPoint3d(0,1,1),0.5,0.5,0.0);
    SmObjDelete sCU3(pLine);
    SER(my_test_GlobalCSI(*pCylBSS1,*pLine,SM_ZONE_TOL_3D,lNumFound));
    SM_ASSERT(lNumFound == 1); // Should be one coincident segment

    SmBSplineCurve *pLine2 = my_create_line(sContext,SmPoint3d(3,1,0),SmPoint3d(3,1,1),0.5,0.5,0.0);
    SmObjDelete sCU4(pLine2);
    SER(my_test_GlobalCSI(*pCylBSS2,*pLine2,SM_ZONE_TOL_3D,lNumFound));
    SM_ASSERT(lNumFound == 1); // Should be one coincident segment
  }

  {
    // Test lines which are on left side but shorter
    SmBSplineCurve *pLine = my_create_line(sContext,SmPoint3d(-1,0,0.2),SmPoint3d(-1,0,0.8),0.5,0.5,0.0);
    SmObjDelete sCU3(pLine);
    SER(my_test_GlobalCSI(*pCylBSS1,*pLine,SM_ZONE_TOL_3D,lNumFound));
    SM_ASSERT(lNumFound == 1); // Should be one coincident segment

    SmBSplineCurve *pLine2 = my_create_line(sContext,SmPoint3d(2,0,0.2),SmPoint3d(2,0,0.8),0.5,0.5,0.0);
    SmObjDelete sCU4(pLine2);
    SER(my_test_GlobalCSI(*pCylBSS2,*pLine2,SM_ZONE_TOL_3D,lNumFound));
    SM_ASSERT(lNumFound == 1); // Should be one coincident segment
  }

  {
    // Test lines which are on right side but longer - also along seam
    SmBSplineCurve *pLine = my_create_line(sContext,SmPoint3d(1,0,-0.2),SmPoint3d(1,0,1.2),0.5,0.5,0.0);
    SmObjDelete sCU3(pLine);
    SER(my_test_GlobalCSI(*pCylBSS1,*pLine,SM_ZONE_TOL_3D,lNumFound));
    SM_ASSERT(lNumFound == 2); // Should be two coincident segment

    SmBSplineCurve *pLine2 = my_create_line(sContext,SmPoint3d(4,0,-0.2),SmPoint3d(4,0,1.2),0.5,0.5,0.0);
    SmObjDelete sCU4(pLine2);
    SER(my_test_GlobalCSI(*pCylBSS2,*pLine2,SM_ZONE_TOL_3D,lNumFound));
    SM_ASSERT(lNumFound == 2); // Should be two coincident segment
  }

  {
    // Test circles at ends
    SmBSplineCurve *pCircle1 = my_create_circle(sContext,1.0,SmPoint3d(0,0,0),SM_CO_QUADRATIC,0,1,1);
    SmObjDelete sCU3(pCircle1);
    SER(my_test_GlobalCSI(*pCylBSS1,*pCircle1,SM_ZONE_TOL_3D,lNumFound));
    SM_ASSERT(lNumFound == 1); // Should be one coincident segment

    SmBSplineCurve *pCircle2 = my_create_circle(sContext,1.0,SmPoint3d(3,0,0),SM_CO_QUADRATIC,0,1,1);
    SmObjDelete sCU4(pCircle2);
    SER(my_test_GlobalCSI(*pCylBSS2,*pCircle2,SM_ZONE_TOL_3D,lNumFound));
    SM_ASSERT(lNumFound == 1); // Should be one coincident segment
  }

  {
    // Test circles in middle
    SmBSplineCurve *pCircle1 = my_create_circle(sContext,1.0,SmPoint3d(0,0,0.3),SM_CO_QUADRATIC,0,1,1);
    SmObjDelete sCU3(pCircle1);
    SER(my_test_GlobalCSI(*pCylBSS1,*pCircle1,SM_ZONE_TOL_3D,lNumFound));
    SM_ASSERT(lNumFound == 1); // Should be one coincident segment

    SmBSplineCurve *pCircle2 = my_create_circle(sContext,1.0,SmPoint3d(3,0,0.3),SM_CO_QUADRATIC,0,1,1);
    SmObjDelete sCU4(pCircle2);
    SER(my_test_GlobalCSI(*pCylBSS2,*pCircle2,SM_ZONE_TOL_3D,lNumFound));
    SM_ASSERT(lNumFound == 1); // Should be one coincident segment
  }

#ifdef SM_GFX_CODE
  if (smGet_DoGraphics())
    {
      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1); pCylBSS1->DrawUV() ;
      smgfx_SetLook(1,2, 0,1,0); pCylBSS2->DrawUV() ;
      sm_GraphicsLoop();
    }
#endif // SM_GFX_CODE

  // all done
  return SM_SUCCESS;

} // end my_test_csi_coincidence


/***********************************************************************
PURPOSE ---

USAGE NOTES ---
***********************************************************************/
PT_EXPORT SmStatus my_test_cs_nurb(void)
{
//    SmContext sContext(pPool);
    SmContext sContext;

    SmTArray<SmPoint3d> sPnts;
    sPnts.Add(SmPoint3d(0,0,0));
    sPnts.Add(SmPoint3d(1,0.1,0.4));
    sPnts.Add(SmPoint3d(2,0.2,0.3));
    sPnts.Add(SmPoint3d(3,0.3,0));
    sPnts.Add(SmPoint3d(0,1,0.2));
    sPnts.Add(SmPoint3d(1,1,0.5));
    sPnts.Add(SmPoint3d(2,1,0.6));
    sPnts.Add(SmPoint3d(3,1,0.2));
    sPnts.Add(SmPoint3d(0,2,0.3));
    sPnts.Add(SmPoint3d(1,2,0.6));
    sPnts.Add(SmPoint3d(2,2,0.5));
    sPnts.Add(SmPoint3d(3,2,0.4));
    sPnts.Add(SmPoint3d(0,3,0.4));
    sPnts.Add(SmPoint3d(1,3,0.2));
    sPnts.Add(SmPoint3d(2,3,0.1));
    sPnts.Add(SmPoint3d(3,3,0.7));
    sPnts.Add(SmPoint3d(0,4,0.4));
    sPnts.Add(SmPoint3d(1,4,0.2));
    sPnts.Add(SmPoint3d(2,4,0.1));
    sPnts.Add(SmPoint3d(3,4,0.7));
#ifdef SM_GFX_CODE
    smgfx_SetColor(0,0,0);
#endif // SM_GFX_CODE
    SmBSplineSurface *pNewBSS = my_create_crease_surf(sContext,SmPoint3d(0,0,0),sbCreases,0.0,0.0,0.0);
    SmObjDelete sCU1(pNewBSS);

#ifdef SM_GFX_CODE
    smgfx_SetLineWidth(2.0);
#endif // SM_GFX_CODE
    SmCurve *pLine = my_create_line(sContext,SmPoint3d(1.3,0.0,3),SmPoint3d(1.3,8.0,3),0.2,0,0.3);
    SmObjDelete sCU9(pLine);

    SmBSplineCurve *pCircle1 = my_create_circle(sContext,1.0,SmPoint3d(1.3,1.8,3),SM_CO_QUADRATIC,0,0,0);
    SmObjDelete sCU2(pCircle1);
    SmBSplineCurve *pCircle2 = my_create_circle(sContext,1.0,SmPoint3d(0,0,3),SM_CO_QUADRATIC,0,0,0);
    SmObjDelete sCU3(pCircle2);
    SmBSplineCurve *pCircle3 = my_create_circle(sContext,1.0,SmPoint3d(3,3,3),SM_CO_QUADRATIC,0,0,0);
    SmObjDelete sCU4(pCircle3);
    SmBSplineCurve *pCircle4 = my_create_circle(sContext,1.0,SmPoint3d(4,5,3),SM_CO_QUADRATIC,0,0,0);
    SmObjDelete sCU5(pCircle4);
    SmBSplineCurve *pCircle5 = my_create_circle(sContext,1.0,SmPoint3d(0,5,3),SM_CO_QUADRATIC,0,0,0);
    SmObjDelete sCU6(pCircle5);
    SmBSplineCurve *pCircle6 = my_create_circle(sContext,1.0,SmPoint3d(4,0,3),SM_CO_QUADRATIC,0,0,0);
    SmObjDelete sCU7(pCircle6);
#ifdef SM_GFX_CODE
    smgfx_SetLineWidth(1.0);
#endif // SM_GFX_CODE

#ifdef SM_GFX_CODE
    if ( smGet_DoGraphics() )
    { pNewBSS->DrawUV( 4, 4 ); }
#endif // SM_GFX_CODE

    {  // Test mirror of curve
        SmAxis2Placement sMirror;
        sMirror.SetCanonical(SmPoint3d(0,0.0,0),SmVector3d(1,0,0),SmVector3d(0,1,0));
        SmSurface *pBSS;
        SER(pNewBSS->CreateMirrorSurface(sContext,sMirror,pBSS));
        SmObjDelete sClean( pBSS );
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
            smgfx_SetColor(0,0,1);
            pBSS->DrawUV(4,4);

        }
#endif // SM_GFX_CODE
    }



    ULONG lCount;
    SmVector3d sVecs[2];
    sVecs[0] = SmVector3d(0,0,1);
    SmVector3d sVecs2[2];
    sVecs2[0] = SmVector3d(0,0,-1);
    SmVector3d sVecs3[2];
    sVecs3[0] = SmVector3d(0.2,0.2,-1);
    SER(sVecs3[0].Unitize());
#ifdef SM_GFX_CODE
    smgfx_SetColor(1.0,0.0,0.0);
#endif // SM_GFX_CODE
    SER(my_test_cs_solve(*pNewBSS,*pCircle1,SM_SO_3D_SIGNED_DIRECTED_MINIMIZE,sVecs2,lCount));
    SM_ASSERT(lCount == 1);
    SER(my_test_cs_solve(*pNewBSS,*pLine,SM_SO_3D_SIGNED_DIRECTED_MINIMIZE,sVecs2,lCount));
    SM_ASSERT(lCount == 1);
    SER(my_test_cs_solve(*pNewBSS,*pCircle1,SM_SO_3D_SIGNED_DIRECTED_MINIMIZE,sVecs,lCount));
    SM_ASSERT(lCount == 1);
    SER(my_test_cs_solve(*pNewBSS,*pLine,SM_SO_3D_SIGNED_DIRECTED_MINIMIZE,sVecs,lCount));
    SM_ASSERT(lCount == 1);
    SER(my_test_cs_solve(*pNewBSS,*pCircle1,SM_SO_3D_SIGNED_DIRECTED_MINIMIZE,sVecs3,lCount));
    SM_ASSERT(lCount == 1);
    SER(my_test_cs_solve(*pNewBSS,*pLine,SM_SO_3D_SIGNED_DIRECTED_MINIMIZE,sVecs3,lCount));
    SM_ASSERT(lCount == 1);

    SER(my_test_cs_solve(*pNewBSS,*pCircle1,SM_SO_MAXIMIZE,sVecs,lCount));
#ifdef SM_GFX_CODE
    smgfx_SetColor(1.0,0.0,1.0);
#endif // SM_GFX_CODE
    SER(my_test_cs_solve(*pNewBSS,*pCircle1,SM_SO_NORMALIZE,sVecs,lCount));
    SM_ASSERT((int)lCount == ( sbCreases ? 7 : 4 ) );

    SER(my_test_cs_solve(*pNewBSS,*pCircle2,SM_SO_MINIMIZE,sVecs,lCount));
    SER(my_test_cs_solve(*pNewBSS,*pCircle2,SM_SO_MAXIMIZE,sVecs,lCount));
#ifdef SM_GFX_CODE
    smgfx_SetColor(1.0,0.0,1.0);
#endif // SM_GFX_CODE

    SER(my_test_cs_solve(*pNewBSS,*pCircle3,SM_SO_MINIMIZE,sVecs,lCount));
    SER(my_test_cs_solve(*pNewBSS,*pCircle3,SM_SO_MAXIMIZE,sVecs,lCount));

    SER(my_test_cs_solve(*pNewBSS,*pCircle4,SM_SO_MINIMIZE,sVecs,lCount));

    SER(my_test_cs_solve(*pNewBSS,*pCircle4,SM_SO_MAXIMIZE,sVecs,lCount));


    SER(my_test_cs_solve(*pNewBSS,*pCircle5,SM_SO_MINIMIZE,sVecs,lCount));

    SER(my_test_cs_solve(*pNewBSS,*pCircle5,SM_SO_MAXIMIZE,sVecs,lCount));


    SER(my_test_cs_solve(*pNewBSS,*pCircle6,SM_SO_MINIMIZE,sVecs,lCount));

    SER(my_test_cs_solve(*pNewBSS,*pCircle6,SM_SO_MAXIMIZE,sVecs,lCount));


//    pPool->Free();
//    delete pPool; pPool = NULL ;
    return SM_SUCCESS;

} // end my_test_cs_nurb

/***********************************************************************
PURPOSE ---  Center the debug graphics window on target surface

USAGE NOTES --- the image will be out of sync with the application's globals
that control view position and orientation.  That means that 
if one goes into the ui_application and tries to modify the screen
view, then one will experience a screen jump as the system switches
from these view parameters to the unupdated global parameters stored in the application
***********************************************************************/
void my_CenterSurface
  (SmSurface *pSurface,               // in : object to center on screen
   double     dScale=.9,              // in : object view scale, .9 = object takes up about 90% of screen
   SmPoint3d *pCenter=NULL)           // in : normalized pSurface->BBox to center in screen
                                      //      ex: [.5,.5,.5] object is centered
                                      //      ex: [.75,.5,.5] object is scooted to the left (the bbox pt to the right is centered)
                                      //      NULL = [.5, .5, .5], default:[NULL]
{
#ifdef SM_GFX_CODE
  SmExtent3d   sBBox ;
  SmPoint3d    sNormCenter = pCenter ? *pCenter : SmPoint3d(.5, .5, .5) ;
  if(pSurface) pSurface->CalculateBoundingBox(pSurface->GetNaturalUVDomain(), &sBBox) ; 

  // when we have a box - center the view on it
  if(!sBBox.IsInit()) 
    { SmPoint3d sCenter = sBBox.Evaluate(.5,.5,.5) ;
      smgfx_SetRotationCenter(sCenter) ;
      smgfx_ZoomWorldBox(sBBox,dScale,&sNormCenter) ;
    }
#else
  SM_REF3(pSurface, dScale, pCenter);
#endif // SM_GFX_CODE

} // end my_CenterSurface

/***********************************************************************
PURPOSE ---  Create and Drop a set of IsoParameter curves to input surface

USAGE NOTES --- creates 6 iso param curves, 
  constand Normalized U[0,.5,1] and
  constant Normalized V[0, .5, 1]

  each curve is dropped to the surface.  The number of results is checked
  and when smGet_DoGraphics() all inputs and outputs are drawn
***********************************************************************/
PT_EXPORT SmStatus my_test_drop_iso_curves(SmBSplineSurface *pNewBSS)    
{
//    SmContext sContext(pPool);
  SmContext sContext;
  const SmContext *pContext =   pNewBSS->GetContext() 
                              ? pNewBSS->GetContext()
                              : &sContext ;
  SmExtent2d sUVDomain = pNewBSS->GetNaturalUVDomain() ;
  SmExtent1d sIvlU     = sUVDomain.GetUInterval() ;
  SmExtent1d sIvlV     = sUVDomain.GetVInterval() ;

#ifdef SM_DEBUG_CODE
  if(smGet_DoGraphics())
    {
      SM_ASSERT_VALID(pNewBSS) ;
    }
#endif // SM_DEBUG_CODE
  
  ULONG ii ;
  double dU, dV ;   
  double dMaxDistToSurf, dDeviation;
  SmTArray<SmBSplineCurve*> sProjCurves;

  // test SM_SP_U[.5]
    {
      SmBSplineCurve *pIsoU = NULL ;
      dU = sIvlU.Evaluate(.5) ;
      SER(pNewBSS->CreateIsoParametricCurve(*pContext,  // in : context for created objects                                         
                                             SM_SP_U,   // in : SM_SP_U = create constant u isoParameter curve                      
                                                        //      SM_SP_V = create constant v isoParameter curve                      
                                             dU,        // in : If eSurfParam==SM_SP_U this is a U parameter, 
                                                        //      if eSurfParam==SM_SP_V this is the V parameter                                        
                                             0.0,       // in : d3DTolerance = Not used in this method                              
                                             pIsoU));   // out: 3D IsoParameterCurve                                                
                                                        // in : optional trim bound for IsoParameterCurve                           
                                                        //      NULL=use GetNaturalUVDomain(), default:[NULL]                       
                                                                                                                                    
      SmObjDelete sClean1(pIsoU);
      if (!pIsoU->IsDegenerate()) 
        {
#ifdef SM_GFX_CODE
          if (smGet_DoGraphics())
            {
              SmPoint3d sPt(.7,.5,.5);
              my_CenterSurface(pNewBSS,.65,&sPt);

              smgfx_Erase() ;
              smgfx_SetLook(1,2, 0,1,1) ; if(pNewBSS) pNewBSS->DrawUV(8,8,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ;
              smgfx_SetLook(3,4, 0,0,1) ; if(pIsoU) pIsoU->Draw(NULL,TRUE) ; sm_GraphicsLoop() ;
              sm_GraphicsLoop() ;
            }
#endif // SM_GFX_CODE

          // drop IsoCurve
          SER(pNewBSS->DropCurve(*pContext,
                                 pNewBSS->GetNaturalUVDomain(),
                                *pIsoU,
                                 pIsoU->GetNaturalInterval(),
                                 SM_ZONE_TOL_3D,
                                 dMaxDistToSurf,   // out: MaxDist(DropPt, 3dCurvePt), found by sampling, may be slightly less than actual max.
                                 dDeviation,       // out: MaxDist(DropPt->SrfNormalLine, 3dCurvePt), Quality Measure: expected to be close to 0.0
                                 sProjCurves));
          SM_ASSERT(sProjCurves.GetSize() == 1);

          for(ii=0;ii<sProjCurves.GetSize();ii++)
            {
              SmBSplineCurve * pCurve = SM_CAST_PTR(SmBSplineCurve,sProjCurves[ii]);
#ifdef SM_GFX_CODE
              if (smGet_DoGraphics())
                {
                  smgfx_SetLook(4,5, 1,0,0) ; if(pCurve) pNewBSS->DrawUVCurve(*pCurve) ; sm_GraphicsLoop() ;
                  sm_GraphicsLoop() ;
                }
#endif // SM_GFX_CODE
              SM_ASSERT(pCurve != NULL) ; delete pCurve ; pCurve = NULL ;
            }
        }

      // test SM_SP_V[.5]
      SmBSplineCurve *pIsoV = NULL ;
      dV = sIvlV.Evaluate(.5) ;
      SER(pNewBSS->CreateIsoParametricCurve(*pContext,SM_SP_V,dV,0.0,pIsoV));
      SmObjDelete sClean2(pIsoV);
      if (!pIsoV->IsDegenerate()) 
        {
#ifdef SM_GFX_CODE
          if (smGet_DoGraphics())
            {
              smgfx_SetLook(1,2, 0,1,1) ; if(pNewBSS) pNewBSS->DrawUV(8,8,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ;
              smgfx_SetLook(3,4, 0,0,1) ; if(pIsoV) pIsoV->Draw(NULL,TRUE) ; sm_GraphicsLoop() ;
              sm_GraphicsLoop() ;
            }
#endif // SM_GFX_CODE

          SER(pNewBSS->DropCurve(*pContext,
                                  pNewBSS->GetNaturalUVDomain(),
                                 *pIsoV,
                                  pIsoV->GetNaturalInterval(),
                                  SM_ZONE_TOL_3D,
                                  dMaxDistToSurf,  // out: MaxDist(DropPt, 3dCurvePt), found by sampling, may be slightly less than actual max.
                                  dDeviation,      // out: MaxDist(DropPt->SrfNormalLine, 3dCurvePt), Quality Measure: expected to be close to 0.0
                                  sProjCurves));
          SM_ASSERT(sProjCurves.GetSize() == 1);

          for(ii=0;ii<sProjCurves.GetSize();ii++)
            {
              SmBSplineCurve * pCurve = SM_CAST_PTR(SmBSplineCurve,sProjCurves[ii]);
#ifdef SM_GFX_CODE
          if (smGet_DoGraphics())
            {
              smgfx_SetLook(4,5, 1,0,0) ; if(pCurve) pNewBSS->DrawUVCurve(*pCurve) ; sm_GraphicsLoop() ;
              sm_GraphicsLoop() ;
            }
#endif // SM_GFX_CODE
              SM_ASSERT(pCurve != NULL) ; delete pCurve ; pCurve = NULL ;
            }
        }

      // test SM_SP_U[0.0]
      dU = sIvlU.Evaluate(0.0) ;
      SER(pNewBSS->CreateIsoParametricCurve(*pContext,SM_SP_U,dU,0.0,pIsoU));
      SmObjDelete sClean3(pIsoU);
      if (!pIsoU->IsDegenerate()) 
        {
#ifdef SM_GFX_CODE
          if (smGet_DoGraphics())
            {
              smgfx_SetLook(1,2, 0,1,1) ; if(pNewBSS) pNewBSS->DrawUV(8,8,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ;
              smgfx_SetLook(3,4, 0,0,1) ; if(pIsoU) pIsoU->Draw(NULL,TRUE) ; sm_GraphicsLoop() ;
              sm_GraphicsLoop() ;
            }
#endif // SM_GFX_CODE

          SER(pNewBSS->DropCurve(*pContext,
                                 pNewBSS->GetNaturalUVDomain(),
                                *pIsoU,pIsoU->GetNaturalInterval(),
                                 SM_ZONE_TOL_3D,
                                 dMaxDistToSurf,   // out: MaxDist(DropPt, 3dCurvePt), found by sampling, may be slightly less than actual max.
                                 dDeviation,       // out: MaxDist(DropPt->SrfNormalLine, 3dCurvePt), Quality Measure: expected to be close to 0.0
                                 sProjCurves));
          SM_ASSERT(sProjCurves.GetSize() >= 1);

          for(ii=0;ii<sProjCurves.GetSize();ii++)
            {
              SmBSplineCurve * pCurve = SM_CAST_PTR(SmBSplineCurve,sProjCurves[ii]);
#ifdef SM_GFX_CODE
          if (smGet_DoGraphics())
            {
              smgfx_SetLook(4,5, 1,0,0) ; if(pCurve) pNewBSS->DrawUVCurve(*pCurve) ; sm_GraphicsLoop() ;
              sm_GraphicsLoop() ;
            }
#endif // SM_GFX_CODE
              SM_ASSERT(pCurve != NULL) ; delete pCurve ; pCurve = NULL ;
            }
        }

      // test IS_SP_V[0.0]
      dV = sIvlV.Evaluate(0.0) ;
      SER(pNewBSS->CreateIsoParametricCurve(*pContext,SM_SP_V,dV,0.0,pIsoV));
      SmObjDelete sClean4(pIsoV);
      if (!pIsoV->IsDegenerate()) 
        {
#ifdef SM_GFX_CODE
          if (smGet_DoGraphics())
            {
              smgfx_SetLook(1,2, 0,1,1) ; if(pNewBSS) pNewBSS->DrawUV(8,8,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ;
              smgfx_SetLook(3,4, 0,0,1) ; if(pIsoV) pIsoV->Draw(NULL,TRUE) ; sm_GraphicsLoop() ;
              sm_GraphicsLoop() ;
            }
#endif // SM_GFX_CODE

          SER(pNewBSS->DropCurve(*pContext,
                                 pNewBSS->GetNaturalUVDomain(),
                                *pIsoV,
                                 pIsoV->GetNaturalInterval(),
                                 SM_ZONE_TOL_3D,
                                 dMaxDistToSurf,   // out: MaxDist(DropPt, 3dCurvePt), found by sampling, may be slightly less than actual max.
                                 dDeviation,       // out: MaxDist(DropPt->SrfNormalLine, 3dCurvePt), Quality Measure: expected to be close to 0.0
                                 sProjCurves));
          SM_ASSERT(sProjCurves.GetSize() >= 1);

          for(ii=0;ii<sProjCurves.GetSize();ii++)
            {
              SmBSplineCurve * pCurve = SM_CAST_PTR(SmBSplineCurve,sProjCurves[ii]);
#ifdef SM_GFX_CODE
          if (smGet_DoGraphics())
            {
              smgfx_SetLook(4,5, 1,0,0) ; if(pCurve) pNewBSS->DrawUVCurve(*pCurve) ; sm_GraphicsLoop() ;
              sm_GraphicsLoop() ;
            }
#endif // SM_GFX_CODE
              SM_ASSERT(pCurve != NULL) ; delete pCurve ; pCurve = NULL ;
            }
        }

      // test SM_SP_U[1.0]
      dU = sIvlU.Evaluate(1.0) ;
      SER(pNewBSS->CreateIsoParametricCurve(*pContext,SM_SP_U,dU,0.0,pIsoU));
      SmObjDelete sClean5(pIsoU);
      if (!pIsoU->IsDegenerate()) 
        {
#ifdef SM_GFX_CODE
          if (smGet_DoGraphics())
            {
              smgfx_SetLook(1,2, 0,1,1) ; if(pNewBSS) pNewBSS->DrawUV(8,8,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ;
              smgfx_SetLook(3,4, 0,0,1) ; if(pIsoU) pIsoU->Draw(NULL,TRUE) ; sm_GraphicsLoop() ;
              sm_GraphicsLoop() ;
            }
#endif // SM_GFX_CODE

          SER(pNewBSS->DropCurve(*pContext,
                                 pNewBSS->GetNaturalUVDomain(),
                                *pIsoU,
                                 pIsoU->GetNaturalInterval(),
                                 SM_ZONE_TOL_3D,
                                 dMaxDistToSurf,  // out: MaxDist(DropPt, 3dCurvePt), found by sampling, may be slightly less than actual max.
                                 dDeviation,      // out: MaxDist(DropPt->SrfNormalLine, 3dCurvePt), Quality Measure: expected to be close to 0.0
                                 sProjCurves));
          SM_ASSERT(sProjCurves.GetSize() >= 1);

          for(ii=0;ii<sProjCurves.GetSize();ii++)
            {
              SmBSplineCurve * pCurve = SM_CAST_PTR(SmBSplineCurve,sProjCurves[ii]);
#ifdef SM_GFX_CODE
          if (smGet_DoGraphics())
            {
              smgfx_SetLook(4,5, 1,0,0) ; if(pCurve) pNewBSS->DrawUVCurve(*pCurve) ; sm_GraphicsLoop() ;
              sm_GraphicsLoop() ;
            }
#endif // SM_GFX_CODE
              SM_ASSERT(pCurve != NULL) ; delete pCurve ; pCurve = NULL ;
            }
        }

      // test SM_SP_V[1.0]
      dV = sIvlV.Evaluate(1.0) ;
      SER(pNewBSS->CreateIsoParametricCurve(*pContext,SM_SP_V,dV,0.0,pIsoV));
      SmObjDelete sClean6(pIsoV);
      if (!pIsoV->IsDegenerate()) 
        {
#ifdef SM_GFX_CODE
          if (smGet_DoGraphics())
            {
              smgfx_SetLook(1,2, 0,1,1) ; if(pNewBSS) pNewBSS->DrawUV(8,8,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ;
              smgfx_SetLook(3,4, 0,0,1) ; if(pIsoV) pIsoV->Draw(NULL,TRUE) ; sm_GraphicsLoop() ;
              sm_GraphicsLoop() ;
            }
#endif // SM_GFX_CODE

          SER(pNewBSS->DropCurve(*pContext,
                                 pNewBSS->GetNaturalUVDomain(),
                                *pIsoV,
                                 pIsoV->GetNaturalInterval(),
                                 SM_ZONE_TOL_3D,
                                 dMaxDistToSurf,   // out: MaxDist(DropPt, 3dCurvePt), found by sampling, may be slightly less than actual max.
                                 dDeviation,       // out: MaxDist(DropPt->SrfNormalLine, 3dCurvePt), Quality Measure: expected to be close to 0.0
                                 sProjCurves));
          SM_ASSERT(sProjCurves.GetSize() >= 1);

          for(ii=0;ii<sProjCurves.GetSize();ii++)
            {
              SmBSplineCurve * pCurve = SM_CAST_PTR(SmBSplineCurve,sProjCurves[ii]);
#ifdef SM_GFX_CODE
          if (smGet_DoGraphics())
            {
              smgfx_SetLook(4,5, 1,0,0) ; if(pCurve) pNewBSS->DrawUVCurve(*pCurve) ; sm_GraphicsLoop() ;
              sm_GraphicsLoop() ;
            }
#endif // SM_GFX_CODE
              SM_ASSERT(pCurve != NULL) ; delete pCurve ; pCurve = NULL ;
            }
        }
    }

//    pPool->Free();
//    delete pPool; pPool = NULL ;
  return SM_SUCCESS;

} // end my_test_drop_iso_curves

/***********************************************************************
PURPOSE --- Compare number of curves in rProjCurves with expected
 lCount and display and delete all curves in rProjCurves.

USAGE NOTES ---
***********************************************************************/
static SmStatus my_test_drop_results
 (const SmContext           & crContext,     // in : context for temp object construction
  const SmSurface           * cpSurface,     // in : DropCurve target surface
  SmTArray<SmBSplineCurve*> & rProjCurves,   // in : DropCurve set of UVCurve results
  ULONG                       lCount)        // in : expected number of curves in rProjCurves
{
  // verify rProjCurves count
  SM_ASSERT(lCount == rProjCurves.GetSize());

#ifdef SM_GFX_CODE
  if(smGet_DoGraphics())
    {
      SM_ASSERT_VALID(cpSurface) ;

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,1,1) ; if(cpSurface) cpSurface->DrawUV(8,8,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }        
#endif // SM_GFX_CODE

  // for every ProjCurve - display, lift and delete
  for (ULONG i=0; i<rProjCurves.GetSize(); i++) 
    {
      SmBSplineCurve *pCurve = SM_CAST_PTR(SmBSplineCurve,rProjCurves[i]);

#ifdef SM_GFX_CODE
  if(smGet_DoGraphics())
    {
      SM_ASSERT_VALID(pCurve) ;
      if(i==10) 
        { smgfx_Erase() ;
          smgfx_SetLook(1,2, 0,1,1) ; if(cpSurface) cpSurface->DrawUV(8,8,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ; 
        }
      smgfx_SetLook(3,4, 0,0,1) ; if(pCurve && cpSurface) cpSurface->DrawUVCurve(*pCurve); sm_GraphicsLoop() ; 
      smgfx_SetLook(1,2, 0,0,1) ; if(pCurve && cpSurface) cpSurface->DrawUVCurve(*pCurve, TRUE); sm_GraphicsLoop() ; 
      sm_GraphicsLoop() ;
    }        
#endif // SM_GFX_CODE

      double dMaxDist;
      SmBSplineCurve *p3dCurve = NULL ;
      SER(cpSurface->LiftCurve(crContext,                       // in : context for new object construction                          
                               cpSurface->GetNaturalUVDomain(), // in : this surface limit                                           
                              *pCurve,                          // in : 2d Parameter curve defined in surface parameter space to lift
                               pCurve->GetNaturalInterval(),    // in : curve segment to lift                                        
                               1.0e-2,                          // in : max allowed distance between output curve and Surface        
                               dMaxDist,                        // out: max dist from output curve to surface                        
                               p3dCurve));                      // out: 3d Curve = Surface(crUVCurveToLift(crInterval))              
                                                                // in : TRUE = this Surface is at least C1 continuous and            
                                                                //             internal surface C0 continuity checks are skipped.    
                                                                //      FALSE= check Surface for C0 discontinuities -                
                                                                //             break lifted curve at each such point, default:[FALSE]                 
#ifdef SM_GFX_CODE
  if(smGet_DoGraphics())
    {
      SM_ASSERT_VALID(pCurve) ;

      smgfx_SetLook(5,6, 1,0,0) ; if(p3dCurve) p3dCurve->Draw(NULL, TRUE) ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }        
#endif // SM_GFX_CODE

      if(p3dCurve) { delete p3dCurve; p3dCurve = NULL ; }
      if(pCurve)   { delete pCurve;   pCurve   = NULL ; }
    
    } // end iter every rProjCurves member

  // all done
  return SM_SUCCESS;

} // end my_test_drop_results

/***********************************************************************
PURPOSE ---

USAGE NOTES ---
***********************************************************************/
// A nearly axial line can cross the drop tolerance of a cylinder seam. A UV
// drop must not connect opposite seam representations through the surface.
// A helix must also keep valid interior points instead of snapping them to a seam.
static SmStatus my_test_drop_curve_near_seam()
{
    SmContext context;
    const double tolerance = 1.0e-5;
    SmBoolean allOkay = TRUE;
    for (int swap = 0; swap < 2; ++swap)
    {
        SmCylinder* surface = new (context)
            SmCylinder(SmPoint3d(0, 0, 0), SmVector3d(1, 0, 0), SmVector3d(0, 1, 0), 0.25, 0, 360, 20, swap, FALSE, FALSE);
        SmObjDelete deleteSurface(surface);
        for (int kind = 0; kind < 5; ++kind)
        {
            for (int reverse = 0; reverse < 2; ++reverse)
            {
                const SmPoint3d start(0.25, 0, 1);
                const SmPoint3d end(0.25, (kind - 1) * 2.0e-5, 18);
                SmBSplineCurve* curve3d = NULL;
                if (kind == 3)
                {
                    // A legitimate long step around the cylinder must remain valid.
                    curve3d = new (context)
                        SmCircle(SmPoint3d(0, 0, 1), SmVector3d(1, 0, 0), SmVector3d(0, 1, 0), SmExtent1d(0, 270), 0.25, 3, NULL, NULL, reverse);
                }
                else if (kind == 4)
                {
                    SER(SmBSplineCurve::CreateHelixSegment(context, SmAxis2Placement(), 20, 0.25, 0.25, 0.25, TRUE, 1.0e-7, curve3d));
                    if (reverse)
                    {
                        SmExtent1d interval;
                        SER(curve3d->ReverseParameterization(curve3d->GetNaturalInterval(), interval));
                    }
                }
                else
                {
                    SER(SmBSplineCurve::CreateLineSegment(context, 3, reverse ? end : start, reverse ? start : end, curve3d));
                }
                SmObjDelete deleteCurve3d(curve3d);
                SmTArray<SmBSplineCurve*> curves;
                double distance, deviation;
                // Assertions and warnings are failures even when adaptive refinement
                // eventually recovers a geometrically accurate result.
                static thread_local SmBoolean hadDiagnostic;
                hadDiagnostic = FALSE;
                const SmErrorCallbackFunctionPtr previousCallback = SmThreadLocalStorage::GetErrCallbackFunction();
                smos_SetErrorCallback([](SmStatus status, const TCHAR*, ULONG, const TCHAR*)
                {
                    if (status == SM_ERR_ASSERT_FAILURE || status == SM_ERR_WARNING)
                    {
                        hadDiagnostic = TRUE;
                    }
                });
                const SmStatus status = surface->DropCurve(
                    context,
                    surface->GetNaturalUVDomain(),
                    *curve3d,
                    curve3d->GetNaturalInterval(),
                    tolerance,
                    distance,
                    deviation,
                    curves,
                    FALSE
                );
                smos_SetErrorCallback(previousCallback);
                SER(status);
                SmBoolean okay = !hadDiagnostic && curves.GetSize() != 0 && (kind != 4 || curves.GetSize() == 1);
                for (ULONG i = 0; i < curves.GetSize(); ++i)
                {
                    SmObjDelete deleteCurve(curves[i]);
                    SmTArray<double> knots;
                    SER(curves[i]->GetKnotsAll(knots));
                    // Sample every knot span: uniform samples of the whole curve
                    // can miss the very short span containing the bad seam jump.
                    for (ULONG k = 1; k < knots.GetSize(); ++k)
                    {
                        if (knots[k] <= knots[k - 1])
                        {
                            continue;
                        }
                        for (int sample = 0; sample <= 4; ++sample)
                        {
                            const double t = knots[k - 1] + (knots[k] - knots[k - 1]) * sample / 4.0;
                            SmPoint3d uv, lifted, original;
                            SER(curves[i]->EvaluatePoint(t, uv));
                            SER(surface->EvaluatePoint(SmPoint2d(uv.x, uv.y), lifted));
                            SER(curve3d->EvaluatePoint(t, original));
                            if (lifted.DistanceBetween(original) > 1.01 * tolerance)
                            {
                                okay = FALSE;
                            }
                        }
                    }
                }
                SM_ASSERT_MSG(okay, _T("Cylinder drop must lift within tolerance without assertions or warnings"));
                if (!okay)
                {
                    allOkay = FALSE;
                }
            }
        }
    }
    return allOkay ? SM_SUCCESS : SM_ERR;
}

PT_EXPORT SmStatus my_test_drop_curve()
{
  SER(my_test_drop_curve_near_seam());
//    SmContext sContext(pPool);
  ULONG ii ;
  SmContext sContext;
  double dMaxDistToSurf, dDeviation;

  // 
  if (FALSE) 
    {
      SmBSplineCurve   * pCurveToDrop = NULL ;
      SmBSplineSurface * pSurface     = NULL ;

      SER(SmBSplineCurve::ReadFromFile  (sContext, _T("curveDropped.txt"),  pCurveToDrop));
      SER(SmBSplineSurface::ReadFromFile(sContext, _T("surfaceToDrop.txt"), pSurface));
      SmObjDelete sClean1(pCurveToDrop);
      SmObjDelete sClean2(pSurface);
      SmTArray<SmBSplineCurve*> sProjCurves;

#ifdef SM_GFX_CODE
      if(smGet_DoGraphics())
        {
          SM_ASSERT_VALID(pSurface) ;
          SM_ASSERT_VALID(pCurveToDrop) ;
          SmPoint3d sPt(.7,.5,.5);
          my_CenterSurface(pSurface,.65,&sPt);

          smgfx_Erase() ;
          smgfx_SetLook(1,2, 0,1,1) ; if(pSurface) pSurface->DrawUV(8,8,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ;
          smgfx_SetLook(2,3, 0,0,1) ; if(pCurveToDrop) pCurveToDrop->Draw(NULL, TRUE) ; sm_GraphicsLoop() ;
          sm_GraphicsLoop() ;
        }        
#endif // SM_GFX_CODE

      SER(pSurface->DropAndTrimCurve
           ( sContext,                          // in : context for newly created geometry                                                   
             pSurface->GetNaturalUVDomain(),    // in : Domain limits for accepting the dropped curve                                        
            *pCurveToDrop,                      // in : Curve to project onto the surface                                                    
             pCurveToDrop->GetNaturalInterval(),// in : cr3dCurve interval to drop                                                           
             1.0,                               // in : max allowed drop distance.                                                           
                                                //      The projected curve is broken up into more than one piece                            
                                                //      everytime the projection curve goes outside the surface boundary                     
                                                //      by more than this amount.                                                            
             dMaxDistToSurf,                    // out: Max dist between 3dCurve and output UVTrimCurve projected surface image.             
                                                //      This is the InputCurve/Surface distance.                                             
             dDeviation,                        // out: ApproxDist = Max gap size found between 3dCurvePts and DroppedCurve->SrfNormalLines. 
                                                //      In most cases it will be close to the dApproxTol.                                    
                                                //      In some cases it will be much better than the dApproxTol.                            
             sProjCurves));                     // out: Projected UVTrimCurves                                                               
                                                // in : TRUE=Only keep curves when rdMaxDropToSurf <= 10*dApproxTol,                         
                                                //      default:[FALSE]                                                                      
                                                // in : TRUE=cr3dCurve is known to lie on the Surface                                        
                                                //      default:[FALSE]                                                                      

      // draw, lift, and delete all drop result curves
      SER(my_test_drop_results(sContext,pSurface,sProjCurves,1));
#ifdef SM_GFX_CODE
      if(smGet_DoGraphics())
        { sm_GraphicsLoop() ; }        
#endif // SM_GFX_CODE
    }

  // drop curve to sub-piece of surface natural boundary
  if (TRUE) 
    {
      SmBSplineCurve   * pCurveToDrop = NULL ;
      SmBSplineSurface * pSurface     = NULL ;
      SER(SmBSplineCurve::ReadFromFile  (sContext, _T("../../TestFiles/pt_TestFiles/Drop/PS10_drp_Crv1.dat"), pCurveToDrop));
      SER(SmBSplineSurface::ReadFromFile(sContext, _T("../../TestFiles/pt_TestFiles/Drop/PS10_drp_Srf1.dat"), pSurface));
      SmObjDelete sClean1(pCurveToDrop);
      SmObjDelete sClean2(pSurface);
      SmTArray<SmBSplineCurve*> sProjCurves;

#ifdef SM_GFX_CODE
      if(smGet_DoGraphics())
        {
          SM_ASSERT_VALID(pSurface) ;
          SM_ASSERT_VALID(pCurveToDrop) ;
          SmPoint3d sPt(.7,.5,.5);
          my_CenterSurface(pSurface,.65,&sPt);

          smgfx_Erase() ;
          smgfx_SetLook(1,2, 0,1,1) ; if(pSurface) pSurface->DrawUV(8,8,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ;
          smgfx_SetLook(3,4, 0,0,1) ; if(pCurveToDrop) pCurveToDrop->Draw(NULL, TRUE) ; sm_GraphicsLoop() ;
          sm_GraphicsLoop() ;
        }        
#endif // SM_GFX_CODE

      SER(pSurface->DropAndTrimCurve(sContext,pSurface->GetNaturalUVDomain(),
                                    *pCurveToDrop,pCurveToDrop->GetNaturalInterval(),
                                     1.0e-2,dMaxDistToSurf,dDeviation,sProjCurves));

      // draw, lift, and delete all drop result curves
      SER(my_test_drop_results(sContext,pSurface,sProjCurves,1));
#ifdef SM_GFX_CODE
      if(smGet_DoGraphics())
        { sm_GraphicsLoop() ; }        
#endif // SM_GFX_CODE
    }

  // nonIsoCurve drop to Large aspect ratio surface (large radius, long slender circular face)
  if (TRUE) 
    {
      SmBSplineCurve   *pCurveToDrop = NULL ;
      SmBSplineSurface *pSurface     = NULL ;
      SER(SmBSplineCurve::ReadFromFile  (sContext, _T("../../TestFiles/pt_TestFiles/Drop/PS10_drp_Crv2.dat"), pCurveToDrop));
      SER(SmBSplineSurface::ReadFromFile(sContext, _T("../../TestFiles/pt_TestFiles/Drop/PS10_drp_Srf2.dat"), pSurface));
      SmObjDelete sClean1(pCurveToDrop);
      SmObjDelete sClean2(pSurface);
      SmTArray<SmBSplineCurve*> sProjCurves;

#ifdef SM_GFX_CODE
      if(smGet_DoGraphics())
        {
          SM_ASSERT_VALID(pSurface) ;
          SM_ASSERT_VALID(pCurveToDrop) ;
          SmPoint3d sPt(.7,.5,.5);
          my_CenterSurface(pSurface,.65,&sPt);

          smgfx_Erase() ;
          smgfx_SetLook(1,2, 0,1,1) ; if(pSurface) pSurface->DrawUV(8,8,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ;
          smgfx_SetLook(3,4, 0,0,1) ; if(pCurveToDrop) pCurveToDrop->Draw(NULL, TRUE) ; sm_GraphicsLoop() ;
          sm_GraphicsLoop() ;
        }        
#endif // SM_GFX_CODE

      SER(pSurface->DropAndTrimCurve(sContext, pSurface->GetNaturalUVDomain(),
                                    *pCurveToDrop, pCurveToDrop->GetNaturalInterval(),
                                     1.0e-2, dMaxDistToSurf, dDeviation, sProjCurves));

      // draw, lift, and delete all drop result curves
      SER(my_test_drop_results(sContext,pSurface,sProjCurves,1));
#ifdef SM_GFX_CODE
      if(smGet_DoGraphics())
        { sm_GraphicsLoop() ; }        
#endif // SM_GFX_CODE
    }

  // small cylinder with curve ending on natural boundary
  if (TRUE) 
    {
      SmBSplineCurve   * pCurveToDrop = NULL ;
      SmBSplineSurface * pSurface     = NULL ;
      SER(SmBSplineCurve::ReadFromFile(sContext,   _T("../../TestFiles/pt_TestFiles/Drop/PS10_drp_Crv3.dat"), pCurveToDrop));
      SER(SmBSplineSurface::ReadFromFile(sContext, _T("../../TestFiles/pt_TestFiles/Drop/PS10_drp_Srf3.dat"), pSurface));
      SmObjDelete sClean1(pCurveToDrop);
      SmObjDelete sClean2(pSurface);
      SmTArray<SmBSplineCurve*> sProjCurves;

#ifdef SM_GFX_CODE
      if(smGet_DoGraphics())
        {
          SM_ASSERT_VALID(pSurface) ;
          SM_ASSERT_VALID(pCurveToDrop) ;
          SmPoint3d sPt(.7,.5,.5);
          my_CenterSurface(pSurface,.65,&sPt);

          SmExtent3d sBBox ;
          if(pSurface) pSurface->CalculateBoundingBox(pSurface->GetNaturalUVDomain(), &sBBox) ; 
          if(!sBBox.IsInit()) { SmPoint3d sCenter = sBBox.Evaluate(.5,.5,.5) ;
                                smgfx_SetRotationCenter(sCenter) ;
                                smgfx_ZoomWorldBox(sBBox) ;
                              }

          smgfx_Erase() ;
          smgfx_SetLook(1,2, 0,1,1) ; if(pSurface) pSurface->DrawUV(8,8,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ;
          smgfx_SetLook(3,4, 0,0,1) ; if(pCurveToDrop) pCurveToDrop->Draw(NULL, TRUE) ; sm_GraphicsLoop() ;
          sm_GraphicsLoop() ;
        }        
#endif // SM_GFX_CODE

      SER(pSurface->DropAndTrimCurve(sContext,pSurface->GetNaturalUVDomain(),
                                    *pCurveToDrop,pCurveToDrop->GetNaturalInterval(),
                                     1.0e-2,dMaxDistToSurf,dDeviation,sProjCurves));
      
      // draw, lift, and delete all drop result curves
      SER(my_test_drop_results(sContext,pSurface,sProjCurves,1));
#ifdef SM_GFX_CODE
      if(smGet_DoGraphics())
        { sm_GraphicsLoop() ; }        
#endif // SM_GFX_CODE
    }

  // very large aspect ratio surface (thin circumferential slice of a large disk)
  // dropping a very short curve not over the surface - no drop result
  if (TRUE) 
    {  // No curves produced for this test - curve not near surface
      SmBSplineCurve   * pCurveToDrop = NULL ;
      SmBSplineSurface * pSurface     = NULL ;
      SER(SmBSplineCurve::ReadFromFile  (sContext, _T("../../TestFiles/pt_TestFiles/Drop/PS10_drp_Crv4.dat"), pCurveToDrop));
      SER(SmBSplineSurface::ReadFromFile(sContext, _T("../../TestFiles/pt_TestFiles/Drop/PS10_drp_Srf4.dat"), pSurface));
      SmObjDelete sClean1(pCurveToDrop);
      SmObjDelete sClean2(pSurface);
      SmTArray<SmBSplineCurve*> sProjCurves;

#ifdef SM_GFX_CODE
      if(smGet_DoGraphics())
        {
          SM_ASSERT_VALID(pSurface) ;
          SM_ASSERT_VALID(pCurveToDrop) ;
          SmPoint3d sPt(.7,.5,.5);
          my_CenterSurface(pSurface,.65,&sPt);

          smgfx_Erase() ;
          smgfx_SetLook(1,2, 0,1,1) ; if(pSurface) pSurface->DrawUV(8,8,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ;
          smgfx_SetLook(3,4, 0,0,1) ; if(pCurveToDrop) pCurveToDrop->Draw(NULL, TRUE) ; sm_GraphicsLoop() ;
          sm_GraphicsLoop() ;
        }        
#endif // SM_GFX_CODE

      SER(pSurface->DropAndTrimCurve(sContext,pSurface->GetNaturalUVDomain(),
                                    *pCurveToDrop,pCurveToDrop->GetNaturalInterval(),
                                     1.0e-2,dMaxDistToSurf,dDeviation,sProjCurves));

      // draw, lift, and delete all drop result curves
      SER(my_test_drop_results(sContext,pSurface,sProjCurves,0));
#ifdef SM_GFX_CODE
      if(smGet_DoGraphics())
        { sm_GraphicsLoop() ; }        
#endif // SM_GFX_CODE
    }

  // long natural boundary drop to very large aspect surface
  if (TRUE) 
    {
      SmBSplineCurve   * pCurveToDrop = NULL ;
      SmBSplineSurface * pSurface     = NULL ;
      SER(SmBSplineCurve::ReadFromFile(sContext,   _T("../../TestFiles/pt_TestFiles/Drop/PS10_drp_Crv5.dat"), pCurveToDrop));
      SER(SmBSplineSurface::ReadFromFile(sContext, _T("../../TestFiles/pt_TestFiles/Drop/PS10_drp_Srf5.dat"), pSurface));
      SmObjDelete sClean1(pCurveToDrop);
      SmObjDelete sClean2(pSurface);
      SmTArray<SmBSplineCurve*> sProjCurves;

#ifdef SM_GFX_CODE
      if(smGet_DoGraphics())
        {
          SM_ASSERT_VALID(pSurface) ;
          SM_ASSERT_VALID(pCurveToDrop) ;
          SmPoint3d sPt(.7,.5,.5);
          my_CenterSurface(pSurface,.65,&sPt);

          smgfx_Erase() ;
          smgfx_SetLook(1,2, 0,1,1) ; if(pSurface) pSurface->DrawUV(8,8,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ;
          smgfx_SetLook(3,4, 0,0,1) ; if(pCurveToDrop) pCurveToDrop->Draw(NULL, TRUE) ; sm_GraphicsLoop() ;
          sm_GraphicsLoop() ;
        }        
#endif // SM_GFX_CODE

      SER(pSurface->DropAndTrimCurve(sContext,pSurface->GetNaturalUVDomain(),
                                    *pCurveToDrop,pCurveToDrop->GetNaturalInterval(),
                                     1.0e-2,dMaxDistToSurf,dDeviation,sProjCurves));

      // draw, lift, and delete all drop result curves
      SER(my_test_drop_results(sContext,pSurface,sProjCurves,1));
#ifdef SM_GFX_CODE
      if(smGet_DoGraphics())
        { sm_GraphicsLoop() ; }        
#endif // SM_GFX_CODE
    }



  //
  if (FALSE) 
    {  // Parallel projection bug from Gestel
      SmBSplineCurve   * pCurveToProj = NULL ;
      SmBSplineSurface * pSurface     = NULL ;
      SER(SmBSplineCurve::ReadFromFile  (sContext, _T("/Customers/Gestel/Smcurve_proj.txt"), pCurveToProj));
      SER(SmBSplineSurface::ReadFromFile(sContext, _T("/Customers/Gestel/Smsurf_proj.txt"),  pSurface));
      SmObjDelete sClean1(pCurveToProj);
      SmObjDelete sClean2(pSurface);
      SmTArray<SmCurve*> sProjCurves;
      SmObjsDelete<SmCurve*> sCleanArr(&sProjCurves);

#ifdef SM_GFX_CODE
      if(smGet_DoGraphics())
        {
          SM_ASSERT_VALID(pSurface) ;
          SM_ASSERT_VALID(pCurveToProj) ;
          SmPoint3d sPt(.7,.5,.5);
          my_CenterSurface(pSurface,.65,&sPt);

          smgfx_Erase() ;
          smgfx_SetLook(1,2, 0,1,1) ; if(pSurface) pSurface->DrawUV(8,8,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ;
          smgfx_SetLook(3,4, 0,0,1) ; if(pCurveToProj) pCurveToProj->Draw(NULL, TRUE) ; sm_GraphicsLoop() ;
          sm_GraphicsLoop() ;
        }        
#endif // SM_GFX_CODE

      SER(pSurface->CreateParallelProjectionCurves(sContext,pSurface->GetNaturalUVDomain(),
                                                  *pCurveToProj,SmVector3d(0,0,-1), 
                                                   NULL, NULL, NULL,&sProjCurves,NULL));

#ifdef SM_GFX_CODE
      if(smGet_DoGraphics())
        {
          smgfx_Erase() ;
          smgfx_SetLook(1,2, 0,1,1) ; if(pSurface) pSurface->DrawUV(8,8,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ;
          smgfx_SetLook(3,4, 0,0,1) ; if(pCurveToProj) pCurveToProj->Draw(NULL, TRUE) ; sm_GraphicsLoop() ;
          for (ULONG i=0; i<sProjCurves.GetSize(); i++) 
            {
              SmBSplineCurve * p3DCurve = SM_CAST_PTR(SmBSplineCurve,sProjCurves[i]);
              p3DCurve->DrawWithKnots();
              if(sProjCurves[i]) { delete sProjCurves[i] ; sProjCurves.SetAt(i, NULL) ; }
            }
          sm_GraphicsLoop() ;
        }        
#endif // SM_GFX_CODE
    }

  // short isoParameter curve drop to very large aspect ratio surface
  if (TRUE) 
    {
      SmBSplineCurve   * pCurveToDrop = NULL ;
      SmBSplineSurface * pSurface     = NULL ;
      SER(SmBSplineCurve::ReadFromFile(sContext,   _T("../../TestFiles/pt_TestFiles/Drop/PS_LiftCrv_Crv1.dat"), pCurveToDrop));
      SER(SmBSplineSurface::ReadFromFile(sContext, _T("../../TestFiles/pt_TestFiles/Drop/PS_LiftCrv_Srf1.dat"), pSurface));
      SmObjDelete sClean1(pCurveToDrop);
      SmObjDelete sClean2(pSurface);
      SmTArray<SmBSplineCurve*> sProjCurves;
      SmBSplineCurve * p3DCurve = NULL ;

#ifdef SM_GFX_CODE
      if(smGet_DoGraphics())
        {
          SM_ASSERT_VALID(pSurface) ;
          SM_ASSERT_VALID(pCurveToDrop) ;
          SmPoint3d sPt(.7,.5,.5);
          my_CenterSurface(pSurface,.65,&sPt);

          smgfx_Erase() ;
          smgfx_SetLook(1,2, 0,1,1) ; if(pSurface) pSurface->DrawUV(8,8,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ;
          smgfx_SetLook(3,4, 0,0,1) ; if(pCurveToDrop) pSurface->DrawUVCurve(*pCurveToDrop) ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 0,0,1) ; if(pCurveToDrop) pSurface->DrawUVCurve(*pCurveToDrop, TRUE) ; sm_GraphicsLoop() ;
          sm_GraphicsLoop() ;
        }        
#endif // SM_GFX_CODE

      SER(pSurface->LiftCurve(sContext,pSurface->GetNaturalUVDomain(),
                             *pCurveToDrop,pCurveToDrop->GetNaturalInterval(),
                              1.0e-2,dMaxDistToSurf,p3DCurve));
      SmObjDelete sClean3(p3DCurve);

#ifdef SM_GFX_CODE
      if(smGet_DoGraphics())
        {
          SM_ASSERT_VALID(p3DCurve) ;

          smgfx_Erase() ;
          smgfx_SetLook(1,2, 0,1,1) ; if(pSurface) pSurface->DrawUV(8,8,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ;
          smgfx_SetLook(3,4, 0,0,1) ; if(pCurveToDrop) pSurface->DrawUVCurve(*pCurveToDrop) ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 0,0,1) ; if(pCurveToDrop) pSurface->DrawUVCurve(*pCurveToDrop, TRUE) ; sm_GraphicsLoop() ;
          smgfx_SetLook(5,6, 1,0,0) ; if(p3DCurve) p3DCurve->Draw(NULL, TRUE) ; sm_GraphicsLoop() ;
          sm_GraphicsLoop() ;
        }        
#endif // SM_GFX_CODE
    }

  // internal curve lift from normal surface
  if (TRUE) 
    {
      SmBSplineCurve   * pCurveToDrop = NULL ;
      SmBSplineSurface * pSurface     = NULL ;
      SER(SmBSplineCurve::ReadFromFile  (sContext, _T("../../TestFiles/pt_TestFiles/Drop/PS_LiftCrv_Crv2.dat"), pCurveToDrop));
      SER(SmBSplineSurface::ReadFromFile(sContext, _T("../../TestFiles/pt_TestFiles/Drop/PS_LiftCrv_Srf2.dat"), pSurface));
      SmObjDelete sClean1(pCurveToDrop);
      SmObjDelete sClean2(pSurface);
      SmTArray<SmBSplineCurve*> sProjCurves;

#ifdef SM_GFX_CODE
      if(smGet_DoGraphics())
        {
          SM_ASSERT_VALID(pSurface) ;
          SM_ASSERT_VALID(pCurveToDrop) ;
          SmPoint3d sPt(.7,.5,.5);
          my_CenterSurface(pSurface,.65,&sPt);

          smgfx_Erase() ;
          smgfx_SetLook(1,2, 0,1,1) ; if(pSurface) pSurface->DrawUV(8,8,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ;
          smgfx_SetLook(3,4, 0,0,1) ; if(pCurveToDrop) pSurface->DrawUVCurve(*pCurveToDrop) ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 0,0,1) ; if(pCurveToDrop) pSurface->DrawUVCurve(*pCurveToDrop, TRUE) ; sm_GraphicsLoop() ;
          sm_GraphicsLoop() ;
        }        
#endif // SM_GFX_CODE

      SmBSplineCurve * p3DCurve = NULL ;
      SER(pSurface->LiftCurve(sContext,pSurface->GetNaturalUVDomain(),
                             *pCurveToDrop,pCurveToDrop->GetNaturalInterval(),
                              0.005,dMaxDistToSurf,p3DCurve));
      SmObjDelete sClean3(p3DCurve);

#ifdef SM_GFX_CODE
      if(smGet_DoGraphics())
        {
          SM_ASSERT_VALID(p3DCurve) ;

          smgfx_Erase() ;
          smgfx_SetLook(1,2, 0,1,1) ; if(pSurface) pSurface->DrawUV(8,8,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ;
          smgfx_SetLook(3,4, 0,0,1) ; if(pCurveToDrop) pSurface->DrawUVCurve(*pCurveToDrop) ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 0,0,1) ; if(pCurveToDrop) pSurface->DrawUVCurve(*pCurveToDrop, TRUE) ; sm_GraphicsLoop() ;
          smgfx_SetLook(5,6, 1,0,0) ; if(p3DCurve) p3DCurve->Draw(NULL, TRUE) ; sm_GraphicsLoop() ;
          sm_GraphicsLoop() ;
        }        
#endif // SM_GFX_CODE
    }
  
  // internal curve lift from large aspect surface 
  if (TRUE) 
    {
      SmBSplineCurve   * pCurveToDrop = NULL ;
      SmBSplineSurface * pSurface     = NULL ;
      SER(SmBSplineCurve::ReadFromFile  (sContext, _T("../../TestFiles/pt_TestFiles/Drop/PS_LiftCrv_Crv3.dat"), pCurveToDrop));
      SER(SmBSplineSurface::ReadFromFile(sContext, _T("../../TestFiles/pt_TestFiles/Drop/PS_LiftCrv_Srf3.dat"), pSurface));
      SmObjDelete sClean1(pCurveToDrop);
      SmObjDelete sClean2(pSurface);
      SmTArray<SmBSplineCurve*> sProjCurves;

#ifdef SM_GFX_CODE
      if(smGet_DoGraphics())
        {
          SM_ASSERT_VALID(pSurface) ;
          SM_ASSERT_VALID(pCurveToDrop) ;
          SmPoint3d sPt(.7,.5,.5);
          my_CenterSurface(pSurface,.65,&sPt);

          smgfx_Erase() ;
          smgfx_SetLook(1,2, 0,1,1) ; if(pSurface) pSurface->DrawUV(8,8,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ;
          smgfx_SetLook(3,4, 0,0,1) ; if(pCurveToDrop) pSurface->DrawUVCurve(*pCurveToDrop) ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 0,0,1) ; if(pCurveToDrop) pSurface->DrawUVCurve(*pCurveToDrop, TRUE) ; sm_GraphicsLoop() ;
          sm_GraphicsLoop() ;
        }        
#endif // SM_GFX_CODE

      SmBSplineCurve * p3DCurve = NULL ;
      SER(pSurface->LiftCurve(sContext,pSurface->GetNaturalUVDomain(),
                             *pCurveToDrop,pCurveToDrop->GetNaturalInterval(),
                              0.005,dMaxDistToSurf,p3DCurve));
      SmObjDelete sClean3(p3DCurve);

#ifdef SM_GFX_CODE
      if(smGet_DoGraphics())
        {
          SM_ASSERT_VALID(p3DCurve) ;

          smgfx_Erase() ;
          smgfx_SetLook(1,2, 0,1,1) ; if(pSurface) pSurface->DrawUV(8,8,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ;
          smgfx_SetLook(3,4, 0,0,1) ; if(pCurveToDrop) pSurface->DrawUVCurve(*pCurveToDrop) ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 0,0,1) ; if(pCurveToDrop) pSurface->DrawUVCurve(*pCurveToDrop, TRUE) ; sm_GraphicsLoop() ;
          smgfx_SetLook(5,6, 1,0,0) ; if(p3DCurve) p3DCurve->Draw(NULL, TRUE) ; sm_GraphicsLoop() ;
          sm_GraphicsLoop() ;
        }        
#endif // SM_GFX_CODE
    }

  // non linear curve lift from normal surface
  if (TRUE) 
    {
      SmBSplineCurve   * pCurveToDrop = NULL ;
      SmBSplineSurface * pSurface = NULL ;
      SER(SmBSplineCurve::ReadFromFile  (sContext, _T("../../TestFiles/pt_TestFiles/Drop/PS_LiftCrv_Crv4.dat"), pCurveToDrop));
      SER(SmBSplineSurface::ReadFromFile(sContext, _T("../../TestFiles/pt_TestFiles/Drop/PS_LiftCrv_Srf4.dat"), pSurface));
      SmObjDelete sClean1(pCurveToDrop);
      SmObjDelete sClean2(pSurface);
      SmTArray<SmBSplineCurve*> sProjCurves;

#ifdef SM_GFX_CODE
      if(smGet_DoGraphics())
        {
          SM_ASSERT_VALID(pSurface) ;
          SM_ASSERT_VALID(pCurveToDrop) ;
          SmPoint3d sPt(.7,.5,.5);
          my_CenterSurface(pSurface,.65,&sPt);

          smgfx_Erase() ;
          smgfx_SetLook(1,2, 0,1,1) ; if(pSurface) pSurface->DrawUV(8,8,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ;
          smgfx_SetLook(3,4, 0,0,1) ; if(pCurveToDrop) pSurface->DrawUVCurve(*pCurveToDrop) ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 0,0,1) ; if(pCurveToDrop) pSurface->DrawUVCurve(*pCurveToDrop, TRUE) ; sm_GraphicsLoop() ;
          sm_GraphicsLoop() ;
        }        
#endif // SM_GFX_CODE

      SmBSplineCurve * p3DCurve = NULL ;
      SER(pSurface->LiftCurve(sContext,pSurface->GetNaturalUVDomain(),
                             *pCurveToDrop,pCurveToDrop->GetNaturalInterval(),
                              0.005,dMaxDistToSurf,p3DCurve));
      SmObjDelete sClean3(p3DCurve);

#ifdef SM_GFX_CODE
      if(smGet_DoGraphics())
        {
          SM_ASSERT_VALID(p3DCurve) ;

          smgfx_Erase() ;
          smgfx_SetLook(1,2, 0,1,1) ; if(pSurface) pSurface->DrawUV(8,8,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ;
          smgfx_SetLook(3,4, 0,0,1) ; if(pCurveToDrop) pSurface->DrawUVCurve(*pCurveToDrop) ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 0,0,1) ; if(pCurveToDrop) pSurface->DrawUVCurve(*pCurveToDrop, TRUE) ; sm_GraphicsLoop() ;
          smgfx_SetLook(5,6, 1,0,0) ; if(p3DCurve) p3DCurve->Draw(NULL, TRUE) ; sm_GraphicsLoop() ;
          sm_GraphicsLoop() ;
        }        
#endif // SM_GFX_CODE
    }

  // lift a kinked UVTrimCurve on a near perimeter walk of a normal surface to a kinked 3dCurve
  if (TRUE) 
    {
      SmBSplineCurve   * pCurveToDrop = NULL ;
      SmBSplineSurface * pSurface     = NULL ;
      smos_WriteBuffer(_T("my_test_drop_curve: lifting a kinked UVTrimCurve to a kinked 3d curve - expect AssertValid failures")) ; 
      SER(SmBSplineCurve::ReadFromFile(sContext,   _T("../../TestFiles/pt_TestFiles/Drop/PS_lift_crv2.dat"), pCurveToDrop));
      SER(SmBSplineSurface::ReadFromFile(sContext, _T("../../TestFiles/pt_TestFiles/Drop/PS_lift_srf2.dat"), pSurface));
      SmObjDelete sClean1(pCurveToDrop);
      SmObjDelete sClean2(pSurface);
      SmTArray<SmBSplineCurve*> sProjCurves;

#ifdef SM_GFX_CODE
      if(smGet_DoGraphics())
        {
          SM_ASSERT_VALID(pSurface) ;
          SM_ASSERT_VALID(pCurveToDrop) ;
          SmPoint3d sPt(.7,.5,.5);
          my_CenterSurface(pSurface,.65,&sPt);

          smgfx_Erase() ;
          smgfx_SetLook(1,2, 0,1,1) ; if(pSurface) pSurface->DrawUV(8,8,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ;
          smgfx_SetLook(3,4, 0,0,1) ; if(pCurveToDrop) pSurface->DrawUVCurve(*pCurveToDrop) ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 0,0,1) ; if(pCurveToDrop) pSurface->DrawUVCurve(*pCurveToDrop, TRUE) ; sm_GraphicsLoop() ;
          sm_GraphicsLoop() ;
        }        
#endif // SM_GFX_CODE

      SmBSplineCurve * p3DCurve = NULL ;
      SER(pSurface->LiftCurve(sContext,pSurface->GetNaturalUVDomain(),
                             *pCurveToDrop,pCurveToDrop->GetNaturalInterval(),
                              1.0e-2,dMaxDistToSurf,p3DCurve));
      SmObjDelete sClean3(p3DCurve);

#ifdef SM_GFX_CODE
      if(smGet_DoGraphics())
        {
          SM_ASSERT_VALID(p3DCurve) ;

          smgfx_Erase() ;
          smgfx_SetLook(1,2, 0,1,1) ; if(pSurface) pSurface->DrawUV(8,8,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ;
          smgfx_SetLook(3,4, 0,0,1) ; if(pCurveToDrop) pSurface->DrawUVCurve(*pCurveToDrop) ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 0,0,1) ; if(pCurveToDrop) pSurface->DrawUVCurve(*pCurveToDrop, TRUE) ; sm_GraphicsLoop() ;
          smgfx_SetLook(5,6, 1,0,0) ; if(p3DCurve) p3DCurve->Draw(NULL, TRUE) ; sm_GraphicsLoop() ;
          sm_GraphicsLoop() ;
        }        
#endif // SM_GFX_CODE
      smos_WriteBuffer(_T("my_test_drop_curve: End kinked UVTrimCurve test - no more expected Assert failures\n")) ; 
    }

  // lift isoparam segment curve on large aspect ratio swept surface with dramatic turns
  if (TRUE) 
    {
      SmBSplineCurve   * pCurveToDrop = NULL ;
      SmBSplineSurface * pSurface     = NULL ;
      SER(SmBSplineCurve::ReadFromFile(sContext,   _T("../../TestFiles/pt_TestFiles/Drop/PS_lift_crv1.dat"), pCurveToDrop));
      SER(SmBSplineSurface::ReadFromFile(sContext, _T("../../TestFiles/pt_TestFiles/Drop/PS_lift_srf1.dat"), pSurface));
      SmObjDelete sClean1(pCurveToDrop);
      SmObjDelete sClean2(pSurface);
      SmTArray<SmBSplineCurve*> sProjCurves;

#ifdef SM_GFX_CODE
      if(smGet_DoGraphics())
        {
          SM_ASSERT_VALID(pSurface) ;
          SM_ASSERT_VALID(pCurveToDrop) ;
          SmPoint3d sPt(.7,.5,.5);
          my_CenterSurface(pSurface,.65,&sPt);

          smgfx_Erase() ;
          smgfx_SetLook(1,2, 0,1,1) ; if(pSurface) pSurface->DrawUV(8,8,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ;
          smgfx_SetLook(3,4, 0,0,1) ; if(pCurveToDrop) pSurface->DrawUVCurve(*pCurveToDrop) ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 0,0,1) ; if(pCurveToDrop) pSurface->DrawUVCurve(*pCurveToDrop, TRUE) ; sm_GraphicsLoop() ;
          sm_GraphicsLoop() ;
        }        
#endif // SM_GFX_CODE

      SmBSplineCurve * p3DCurve = NULL ;
      SER(pSurface->LiftCurve(sContext,pSurface->GetNaturalUVDomain(),
                             *pCurveToDrop,pCurveToDrop->GetNaturalInterval(),
                              1.0e-2,dMaxDistToSurf,p3DCurve));
      SmObjDelete sClean3(p3DCurve);

#ifdef SM_GFX_CODE
  if(smGet_DoGraphics())
    {
      SM_ASSERT_VALID(p3DCurve) ;

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,1,1) ; if(pSurface) pSurface->DrawUV(8,8,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 0,0,1) ; if(pCurveToDrop) pSurface->DrawUVCurve(*pCurveToDrop) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pCurveToDrop) pSurface->DrawUVCurve(*pCurveToDrop, TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(5,6, 1,0,0) ; if(p3DCurve) p3DCurve->Draw(NULL, TRUE) ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }        
#endif // SM_GFX_CODE
    }

  // drop long curve starting over (drop curve needs to be trimmed) small doubly curved surface
  if (TRUE) 
    {
      SmTArray<SmPoint3d> sPnts;
      my_load_surf1_pts(sPnts);
      SmBSplineSurface *pNewBSS = my_create_nurb_surf(sContext,SmPoint3d(4,0,0),sPnts,6,4,3,3,1.0,0.5,0);
      SmObjDelete sClean6(pNewBSS);

      SmTArray<SmBSplineCurve*> sProjCurves;

      SmBSplineCurve *pLine = my_create_line(sContext,SmPoint3d(4,0,0),SmPoint3d(12,4.2,0.0),1,0,0);
      SmObjDelete sCU1(pLine);

#ifdef SM_GFX_CODE
      if(smGet_DoGraphics())
        {
          SM_ASSERT_VALID(pNewBSS) ;
          SM_ASSERT_VALID(pLine) ;
          SmPoint3d sPt(.7,.5,.5);
          my_CenterSurface(pNewBSS,.65,&sPt);

          smgfx_Erase() ;
          smgfx_SetLook(1,2, 0,1,1) ; if(pNewBSS) pNewBSS->DrawUV(8,8,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ;
          smgfx_SetLook(3,4, 0,0,1) ; if(pLine) pLine->Draw(NULL, TRUE) ; sm_GraphicsLoop() ;
          sm_GraphicsLoop() ;
        }        
#endif // SM_GFX_CODE

      pNewBSS->DropCurve
         (sContext,                       // in : context for new object construction                                                   
          pNewBSS->GetNaturalUVDomain(),  // in : domain of interest for this surface                                                   
         *pLine,                          // in : Curve to project onto the surface                                                     
          pLine->GetNaturalInterval(),    // in : interval of interest for target curve                                                 
          1.0e-2,                         // in : max allowed distance between drop point and SrfNormal line at drop point              
          dMaxDistToSurf,                 // out: MaxDist(DropPt, 3dCurvePt), found by sampling, may be slightly less than actual max.
          dDeviation,                     // out: MaxDist(DropPt->SrfNormalLine, 3dCurvePt), Quality Measure: expected to be close to 0.0  
          sProjCurves);                   // out: 1 (or 2) curves constructed by projection.                                            
                                          //      (2 curves for closed surfaces when cr3dCurve is coincident with seam)                 
                                          // in : TRUE = only return drop curves with drop distances less than dApproxTol               
                                          //      FALSE= return all drop curves                                                         

          for(ii=0;ii<sProjCurves.GetSize();ii++)
            {
              SmBSplineCurve * pCurve = SM_CAST_PTR(SmBSplineCurve,sProjCurves[ii]);
#ifdef SM_GFX_CODE
          if (smGet_DoGraphics())
            {
              SmPoint2d sStartUV ;
              SmPoint3d sPt ;
              if(pCurve) { pCurve->EvaluatePoint(pCurve->GetNaturalInterval().GetMin(), sPt) ;
                           sStartUV.Set(sPt.x, sPt.y) ;
                         }
              smgfx_SetLook(7,8, 0,0,1) ; if(pCurve) pNewBSS->DrawAt(sStartUV) ; sm_GraphicsLoop() ;
              smgfx_SetLook(4,5, 1,0,0) ; if(pCurve) pNewBSS->DrawUVCurve(*pCurve) ; sm_GraphicsLoop() ;
              smgfx_SetLook(1,2, 1,0,0) ; if(pCurve) pNewBSS->DrawUVCurve(*pCurve, TRUE) ; sm_GraphicsLoop() ;
              sm_GraphicsLoop() ;
            }
#endif // SM_GFX_CODE
              SM_ASSERT(pCurve != NULL) ; delete pCurve ; pCurve = NULL ;
            }
    }

  // drop long curve ending over (drop curve needs to be trimmed) small doubly curved surface
  if (TRUE) 
    {
      SmTArray<SmPoint3d> sPnts;
      my_load_surf1_pts(sPnts);
      SmBSplineSurface *pNewBSS = my_create_nurb_surf(sContext,SmPoint3d(4,0,0),sPnts,6,4,3,3,1.0,0.5,0);
      SmObjDelete sClean6(pNewBSS);

      SmTArray<SmBSplineCurve*> sProjCurves;

      SmBSplineCurve *pLine = my_create_line(sContext,SmPoint3d(12,4.2,0.0),SmPoint3d(4,0,0),1,0,0);
      SmObjDelete sCU1(pLine);

#ifdef SM_GFX_CODE
      if(smGet_DoGraphics())
        {
          SM_ASSERT_VALID(pNewBSS) ;
          SM_ASSERT_VALID(pLine) ;
          SmPoint3d sPt(.7,.5,.5);
          my_CenterSurface(pNewBSS,.65,&sPt);

          smgfx_Erase() ;
          smgfx_SetLook(1,2, 0,1,1) ; if(pNewBSS) pNewBSS->DrawUV(8,8,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ;
          smgfx_SetLook(3,4, 0,0,1) ; if(pLine) pLine->Draw(NULL, TRUE) ; sm_GraphicsLoop() ;
          sm_GraphicsLoop() ;
        }        
#endif // SM_GFX_CODE

      pNewBSS->DropCurve
         (sContext,                       // in : context for new object construction                                                   
          pNewBSS->GetNaturalUVDomain(),  // in : domain of interest for this surface                                                   
         *pLine,                          // in : Curve to project onto the surface                                                     
          pLine->GetNaturalInterval(),    // in : interval of interest for target curve                                                 
          1.0e-2,                         // in : max allowed distance between drop point and SrfNormal line at drop point              
          dMaxDistToSurf,                 // out: MaxDist(DropPt, 3dCurvePt), found by sampling, may be slightly less than actual max.
          dDeviation,                     // out: MaxDist(DropPt->SrfNormalLine, 3dCurvePt), Quality Measure: expected to be close to 0.0
          sProjCurves);                   // out: 1 (or 2) curves constructed by projection.                                            
                                          //      (2 curves for closed surfaces when cr3dCurve is coincident with seam)                 
                                          // in : TRUE = only return drop curves with drop distances less than dApproxTol               
                                          //      FALSE= return all drop curves                                                         

          for(ii=0;ii<sProjCurves.GetSize();ii++)
            {
              SmBSplineCurve * pCurve = SM_CAST_PTR(SmBSplineCurve,sProjCurves[ii]);
#ifdef SM_GFX_CODE
          if (smGet_DoGraphics())
            {
              SmPoint2d sStartUV ;
              SmPoint3d sPt ;
              if(pCurve) { pCurve->EvaluatePoint(pCurve->GetNaturalInterval().GetMin(), sPt) ;
                           sStartUV.Set(sPt.x, sPt.y) ;
                         }
              smgfx_SetLook(7,8, 0,0,1) ; if(pCurve) pNewBSS->DrawAt(sStartUV) ; sm_GraphicsLoop() ;
              smgfx_SetLook(4,5, 1,0,0) ; if(pCurve) pNewBSS->DrawUVCurve(*pCurve) ; sm_GraphicsLoop() ;
              smgfx_SetLook(1,2, 1,0,0) ; if(pCurve) pNewBSS->DrawUVCurve(*pCurve, TRUE) ; sm_GraphicsLoop() ;
              sm_GraphicsLoop() ;
            }
#endif // SM_GFX_CODE
              SM_ASSERT(pCurve != NULL) ; delete pCurve ; pCurve = NULL ;
            }
    }

  // test drop isoparameter curves to various cones and cylinders
  if (TRUE) 
    {
      SmBSplineSurface *pCylBSS = NULL ;
      SmAxis2Placement sA2P;

      
      // test closed down pointing cone positioned at [0,-4,0]
      sA2P.SetCanonical(SmPoint3d(0,-4,0),SmVector3d(1,0,0),SmVector3d(0,1,0));
      SER(SmBSplineSurface::CreateConePatch(sContext,
                                            sA2P,1.0,0.0,0.0,360.0,1.0,
                                            SM_CO_QUADRATIC,pCylBSS));
      SmObjDelete sClean1(pCylBSS);
      SER(my_test_drop_iso_curves(pCylBSS));

      
      // test closed cylinder positioned at [4,0,0]
      sA2P.SetCanonical(SmPoint3d(4,0,0),SmVector3d(1,0,0),SmVector3d(0,1,0));
      SER(SmBSplineSurface::CreateConePatch(sContext,
                                            sA2P,1.0,1.0,0.0,360.0,1.0,
                                            SM_CO_QUADRATIC,pCylBSS));
      SmObjDelete sClean2(pCylBSS);
      SER(my_test_drop_iso_curves(pCylBSS));

      
      // test half cylinder positioned at [-4,4,0]
      sA2P.SetCanonical(SmPoint3d(-4,4,0),SmVector3d(1,0,0),SmVector3d(0,1,0));
      SER(SmBSplineSurface::CreateConePatch(sContext,
                                            sA2P,1.0,1.0,0.0,180.0,1.0,
                                            SM_CO_QUADRATIC,pCylBSS));
      SmObjDelete sClean3(pCylBSS);
      SER(my_test_drop_iso_curves(pCylBSS));

      
      // test closed upward pointing cone positioned at [-4,4,0]
      sA2P.SetCanonical(SmPoint3d(4,-4,0),SmVector3d(1,0,0),SmVector3d(0,1,0));
      SER(SmBSplineSurface::CreateConePatch(sContext,
                                            sA2P,0.0,1.0,0.0,360.0,1.0,
                                            SM_CO_QUINTIC,pCylBSS));
      SmObjDelete sClean4(pCylBSS);
      SER(my_test_drop_iso_curves(pCylBSS));

      
      // test closed downward pointing cone positioned at [-4,4,0]
      sA2P.SetCanonical(SmPoint3d(-4,-4,0),SmVector3d(1,0,0),SmVector3d(0,1,0));
      SER(SmBSplineSurface::CreateConePatch(sContext,
                                            sA2P,1.0,0.0,0.0,360.0,1.0,
                                            SM_CO_QUINTIC,pCylBSS));
      SmObjDelete sClean5( pCylBSS );
      SER(my_test_drop_iso_curves(pCylBSS));
    }

      // Test non ISO dropping
  // drop circle to doubly curved surface
  if (TRUE) 
    {
      SmTArray<SmPoint3d> sPnts;
      my_load_surf1_pts(sPnts);
      SmBSplineSurface *pNewBSS = my_create_nurb_surf(sContext,SmPoint3d(4,0,0),sPnts,6,4,3,3,1.0,0.5,0);
      SmObjDelete sClean6(pNewBSS);

      SmBSplineCurve *pCircle = my_create_circle(sContext,1.0,SmPoint3d(5.5,2.0,0.8),
                                               SM_CO_QUINTIC,1,0,0);
      SmObjDelete sCU2(pCircle);
      SmTArray<SmBSplineCurve*> sProjCurves;

#ifdef SM_GFX_CODE
      if(smGet_DoGraphics())
        {
          SM_ASSERT_VALID(pNewBSS) ;
          SM_ASSERT_VALID(pCircle) ;
          SmPoint3d sPt(.7,.5,.5);
          my_CenterSurface(pNewBSS,.65,&sPt);

          smgfx_Erase() ;
          smgfx_SetLook(1,2, 0,1,1) ; if(pNewBSS) pNewBSS->DrawUV(8,8,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ;
          smgfx_SetLook(3,4, 0,0,1) ; if(pCircle) pCircle->Draw(NULL, TRUE) ; sm_GraphicsLoop() ;
          sm_GraphicsLoop() ;
        }        
#endif // SM_GFX_CODE

      SER(pNewBSS->DropCurve(sContext,
                             pNewBSS->GetNaturalUVDomain(),
                            *pCircle,
                             pCircle->GetNaturalInterval(),
                             1.0e-2,
                             dMaxDistToSurf,  // out: MaxDist(DropPt, 3dCurvePt), found by sampling, may be slightly less than actual max.
                             dDeviation,      // out: MaxDist(DropPt->SrfNormalLine, 3dCurvePt), Quality Measure: expected to be close to 0.0
                             sProjCurves));

      // draw, lift, and delete all drop result curves
      SER(my_test_drop_results(sContext,pNewBSS,sProjCurves,1));
      SER(my_test_drop_iso_curves(pNewBSS));
#ifdef SM_GFX_CODE
      if(smGet_DoGraphics())
        { smgfx_SetLook(3,4, 0,0,1) ; if(pCircle) pCircle->Draw(NULL, TRUE) ; sm_GraphicsLoop() ;
          sm_GraphicsLoop() ; 
        }        
#endif // SM_GFX_CODE
      
      // Drop line to doubly curved surface with no trimming
      SmBSplineCurve *pLine = my_create_line(sContext,SmPoint3d(4,0,0),SmPoint3d(7,4.2,0.0),1,0,0);
      SmObjDelete sCU1(pLine);

#ifdef SM_GFX_CODE
      if(smGet_DoGraphics())
        {
          SM_ASSERT_VALID(pNewBSS) ;
          SM_ASSERT_VALID(pLine) ;
          SmPoint3d sPt(.7,.5,.5);
          my_CenterSurface(pNewBSS,.65,&sPt);

          smgfx_Erase() ;
          smgfx_SetLook(1,2, 0,1,1) ; if(pNewBSS) pNewBSS->DrawUV(8,8,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ;
          smgfx_SetLook(3,4, 0,0,1) ; if(pLine) pLine->Draw(NULL, TRUE) ; sm_GraphicsLoop() ;
          sm_GraphicsLoop() ;
        }        
#endif // SM_GFX_CODE

      SER(pNewBSS->DropCurve(sContext,
                             pNewBSS->GetNaturalUVDomain(),
                            *pLine,
                             pLine->GetNaturalInterval(),
                             1.0e-2,    
                             dMaxDistToSurf,  // out: MaxDist(DropPt, 3dCurvePt), found by sampling, may be slightly less than actual max.
                             dDeviation,      // out: MaxDist(DropPt->SrfNormalLine, 3dCurvePt), Quality Measure: expected to be close to 0.0
                             sProjCurves));

      // draw, lift, and delete all drop result curves
      SER(my_test_drop_results(sContext,pNewBSS,sProjCurves,1));

#ifdef SM_GFX_CODE
      if(smGet_DoGraphics())
        { smgfx_SetLook(3,4, 0,0,1) ; if(pLine) pLine->Draw(NULL, TRUE) ; sm_GraphicsLoop() ;
          sm_GraphicsLoop() ; 
        }        
#endif // SM_GFX_CODE
      
      // drop circle to doubly curved surface - no trimming
      pCircle = my_create_circle(sContext,1.0,SmPoint3d(5.3,2.0,0.9),
                                               SM_CO_QUADRATIC,1,0,0);
      SmObjDelete sCU3(pCircle);

#ifdef SM_GFX_CODE
      if(smGet_DoGraphics())
        {
          SM_ASSERT_VALID(pNewBSS) ;
          SM_ASSERT_VALID(pCircle) ;
          SmPoint3d sPt(.7,.5,.5);
          my_CenterSurface(pNewBSS,.65,&sPt);

          smgfx_Erase() ;
          smgfx_SetLook(1,2, 0,1,1) ; if(pNewBSS) pNewBSS->DrawUV(8,8,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ;
          smgfx_SetLook(3,4, 0,0,1) ; if(pCircle) pCircle->Draw(NULL, TRUE) ; sm_GraphicsLoop() ;
          sm_GraphicsLoop() ;
        }        
#endif // SM_GFX_CODE

      SER(pNewBSS->DropCurve(sContext,
                             pNewBSS->GetNaturalUVDomain(),
                            *pCircle,
                             pCircle->GetNaturalInterval(),
                             1.0e-2,
                             dMaxDistToSurf,  // out: MaxDist(DropPt, 3dCurvePt), found by sampling, may be slightly less than actual max.
                             dDeviation,      // out: MaxDist(DropPt->SrfNormalLine, 3dCurvePt), Quality Measure: expected to be close to 0.0
                             sProjCurves));

      // draw, lift, and delete all drop result curves
      SER(my_test_drop_results(sContext,pNewBSS,sProjCurves,1));

#ifdef SM_GFX_CODE
      if(smGet_DoGraphics())
        { smgfx_SetLook(3,4, 0,0,1) ; if(pCircle) pCircle->Draw(NULL, TRUE) ; sm_GraphicsLoop() ;
          sm_GraphicsLoop() ; 
        }        
#endif // SM_GFX_CODE

      // drop line to doubly curved surface - no trimming
      pLine = my_create_line(sContext,SmPoint3d(4,0,0),SmPoint3d(7,4,0.0),1,0,0);
      SmObjDelete sCU4(pLine);

#ifdef SM_GFX_CODE
      if(smGet_DoGraphics())
        {
          SM_ASSERT_VALID(pNewBSS) ;
          SM_ASSERT_VALID(pLine) ;
          SmPoint3d sPt(.7,.5,.5);
          my_CenterSurface(pNewBSS,.65,&sPt);

          smgfx_Erase() ;
          smgfx_SetLook(1,2, 0,1,1) ; if(pNewBSS) pNewBSS->DrawUV(8,8,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ;
          smgfx_SetLook(3,4, 0,0,1) ; if(pLine) pLine->Draw(NULL, TRUE) ; sm_GraphicsLoop() ;
          sm_GraphicsLoop() ;
        }        
#endif // SM_GFX_CODE

      SER(pNewBSS->DropCurve(sContext,
                             pNewBSS->GetNaturalUVDomain(),
                            *pLine,
                             pLine->GetNaturalInterval(),
                             1.0e-2,    
                             dMaxDistToSurf,   // out: MaxDist(DropPt, 3dCurvePt), found by sampling, may be slightly less than actual max.
                             dDeviation,       // out: MaxDist(DropPt->SrfNormalLine, 3dCurvePt), Quality Measure: expected to be close to 0.0
                             sProjCurves));

      // draw, lift, and delete all drop result curves
      SER(my_test_drop_results(sContext,pNewBSS,sProjCurves,1));

#ifdef SM_GFX_CODE
      if(smGet_DoGraphics())
        { smgfx_SetLook(3,4, 0,0,1) ; if(pLine) pLine->Draw(NULL, TRUE) ; sm_GraphicsLoop() ;
          sm_GraphicsLoop() ; 
        }        
#endif // SM_GFX_CODE

    pCircle = my_create_circle(sContext,1.0,SmPoint3d(5.3,2.0,0.9),
                                             SM_CO_QUADRATIC,1,0,0);
    SmObjDelete sCUa(pCircle);
    SER(pNewBSS->DropCurve(sContext,pNewBSS->GetNaturalUVDomain(),*pCircle,
                           pCircle->GetNaturalInterval(),1.0e-2,
                           dMaxDistToSurf,    // out: MaxDist(DropPt, 3dCurvePt), found by sampling, may be slightly less than actual max.
                           dDeviation,        // out: MaxDist(DropPt->SrfNormalLine, 3dCurvePt), Quality Measure: expected to be close to 0.0
                           sProjCurves));

    SER(my_test_drop_results(sContext,pNewBSS,sProjCurves,1));

    pLine = my_create_line(sContext,SmPoint3d(4,0,0),SmPoint3d(7,4,0.0),1,0,0);
    SmObjDelete sCUb(pLine);
    SER(pNewBSS->DropCurve(sContext,pNewBSS->GetNaturalUVDomain(),*pLine,
                           pLine->GetNaturalInterval(),1.0e-2,    
                           dMaxDistToSurf,    // out: MaxDist(DropPt, 3dCurvePt), found by sampling, may be slightly less than actual max.
                           dDeviation,        // out: MaxDist(DropPt->SrfNormalLine, 3dCurvePt), Quality Measure: expected to be close to 0.0
                           sProjCurves));

    SER(my_test_drop_results(sContext,pNewBSS,sProjCurves,1));

      // drop different line to doubly curved surface - no trimming
      pLine = my_create_line(sContext,SmPoint3d(7,0.3,0),SmPoint3d(4,4,0.4),0,0,1);
      SmObjDelete sCU5(pLine);

#ifdef SM_GFX_CODE
      if(smGet_DoGraphics())
        {
          SM_ASSERT_VALID(pNewBSS) ;
          SM_ASSERT_VALID(pLine) ;
          SmPoint3d sPt(.7,.5,.5);
          my_CenterSurface(pNewBSS,.65,&sPt);

          smgfx_Erase() ;
          smgfx_SetLook(1,2, 0,1,1) ; if(pNewBSS) pNewBSS->DrawUV(8,8,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ;
          smgfx_SetLook(3,4, 0,0,1) ; if(pLine) pLine->Draw(NULL, TRUE) ; sm_GraphicsLoop() ;
          sm_GraphicsLoop() ;
        }        
#endif // SM_GFX_CODE

      SER(pNewBSS->DropCurve(sContext,
                             pNewBSS->GetNaturalUVDomain(),
                            *pLine,
                             pLine->GetNaturalInterval(),
                             1.0e-2,    
                             dMaxDistToSurf,  // out: MaxDist(DropPt, 3dCurvePt), found by sampling, may be slightly less than actual max.
                             dDeviation,      // out: MaxDist(DropPt->SrfNormalLine, 3dCurvePt), Quality Measure: expected to be close to 0.0
                             sProjCurves));

      // draw, lift, and delete all drop result curves
      SER(my_test_drop_results(sContext,pNewBSS,sProjCurves,1));

#ifdef SM_GFX_CODE
      if(smGet_DoGraphics())
        { smgfx_SetLook(3,4, 0,0,1) ; if(pLine) pLine->Draw(NULL, TRUE) ; sm_GraphicsLoop() ;
          sm_GraphicsLoop() ; 
        }        
#endif // SM_GFX_CODE
    }

  // drop irregularly shaped closed curve to doubly curved surface - no trimming
  if (TRUE) 
    {
      SmTArray<SmPoint3d> sPnts;
      my_load_surf1_pts(sPnts);
      SmBSplineSurface *pNewBSS = my_create_nurb_surf(sContext,SmPoint3d(0,0,0),sPnts,6,4,3,3,1.0,0.5,0);
      SmObjDelete sClean6(pNewBSS);
      SmTArray<SmPoint3d> sPnts2;
      sPnts2.Add(SmPoint3d(0.5,0.5,0.5));
      sPnts2.Add(SmPoint3d(1.0,1.0,0.5));
      sPnts2.Add(SmPoint3d(2.0,1.3,0.5));
      sPnts2.Add(SmPoint3d(2.7,0.8,0.5));
      sPnts2.Add(SmPoint3d(2.8,0.8,0.5));
      sPnts2.Add(SmPoint3d(2.9,0.8,0.5));
      sPnts2.Add(SmPoint3d(2.8,2.5,0.5));
      sPnts2.Add(SmPoint3d(2.8,2.6,0.5));
      sPnts2.Add(SmPoint3d(2.8,2.8,0.5));
      sPnts2.Add(SmPoint3d(2.0,2.5,0.5));
      sPnts2.Add(SmPoint3d(1.0,3.3,0.5));
      sPnts2.Add(SmPoint3d(0.0,3.5,0.5));
      sPnts2.Add(SmPoint3d(0.5,0.5,0.5));

#ifdef SM_GFX_CODE
      smgfx_SetLineWidth(2.0);
#endif // SM_GFX_CODE

      SmBSplineCurve *pBSC = my_create_nurb(sContext,SmPoint3d(0,0,0),sPnts2,3,0.0,0,1.0);
      SmObjDelete sClean7(pBSC);
      SmTArray<SmBSplineCurve*> sProjCurves;

#ifdef SM_GFX_CODE
      if(smGet_DoGraphics())
        {
          SM_ASSERT_VALID(pNewBSS) ;
          SM_ASSERT_VALID(pBSC) ;
          SmPoint3d sPt(.7,.5,.5);
          my_CenterSurface(pNewBSS,.65,&sPt);

          smgfx_Erase() ;
          smgfx_SetLook(1,2, 0,1,1) ; if(pNewBSS) pNewBSS->DrawUV(8,8,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ;
          smgfx_SetLook(3,4, 0,0,1) ; if(pBSC) pBSC->Draw(NULL, TRUE) ; sm_GraphicsLoop() ;
          sm_GraphicsLoop() ;
        }        
#endif // SM_GFX_CODE

      SER(pNewBSS->DropCurve(sContext,
                             pNewBSS->GetNaturalUVDomain(),
                            *pBSC,
                             pBSC->GetNaturalInterval(),
                             1.0e-2,
                             dMaxDistToSurf,   // out: MaxDist(DropPt, 3dCurvePt), found by sampling, may be slightly less than actual max.
                             dDeviation,       // out: MaxDist(DropPt->SrfNormalLine, 3dCurvePt), Quality Measure: expected to be close to 0.0
                             sProjCurves));
      // draw, lift, and delete all drop result curves
      SER(my_test_drop_results(sContext,pNewBSS,sProjCurves,1));
#ifdef SM_GFX_CODE
      if(smGet_DoGraphics())
        { smgfx_SetLook(3,4, 0,0,1) ; if(pBSC) pBSC->Draw(NULL, TRUE) ; sm_GraphicsLoop() ;
          sm_GraphicsLoop() ; 
        }        
#endif // SM_GFX_CODE
    }

  // lift and drop a diagonal UVCurve from a doubly curved surface
  if (TRUE) 
    { // Test Lift/Drop of curves on surfaces which are C0
      SmBSplineSurface *pNewBSS   = my_create_crease_surf(sContext,SmPoint3d(8,0,0),sbCreases,1.0,0.5,0.0);
      SmObjDelete       sCU1(pNewBSS);
      SmExtent2d        sUVDomain = pNewBSS->GetNaturalUVDomain();
      SmBSplineCurve   *pLine     = my_create_line(sContext,
                                                   SmPoint3d(sUVDomain.GetMin()),
                                                   SmPoint3d(sUVDomain.GetMax()),
                                                   1,0,0,
                                                   TRUE);  // in : TRUE = output 2d Line
      SmObjDelete sCU2(pLine);
      SmBSplineCurve *p3DCurve = NULL ;
      double dMaxDist;

#ifdef SM_GFX_CODE
      if(smGet_DoGraphics())
        {
          SM_ASSERT_VALID(pNewBSS) ;
          SM_ASSERT_VALID(pLine) ;
          SmPoint3d sPt(.7,.5,.5);
          my_CenterSurface(pNewBSS,.65,&sPt);

          smgfx_Erase() ;
          smgfx_SetLook(1,2, 0,1,1) ; if(pNewBSS) pNewBSS->DrawUV(8,8,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ;
          smgfx_SetLook(3,4, 0,0,1) ; if(pLine) pNewBSS->DrawUVCurve(*pLine) ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 0,0,1) ; if(pLine) pNewBSS->DrawUVCurve(*pLine, TRUE) ; sm_GraphicsLoop() ;
          sm_GraphicsLoop() ;
        }        
#endif // SM_GFX_CODE

      SER(pNewBSS->LiftCurve(sContext,
                             sUVDomain,
                             *pLine,
                             pLine->GetNaturalInterval(),
                             1.0e-3,
                             dMaxDist,
                             p3DCurve));

      SmTArray<SmBSplineCurve*> sProjCurves;

      SER(pNewBSS->DropCurve(sContext,
                             pNewBSS->GetNaturalUVDomain(),
                            *p3DCurve,
                             p3DCurve->GetNaturalInterval(),
                             1.0e-2,
                             dMaxDistToSurf,  // out: MaxDist(DropPt, 3dCurvePt), found by sampling, may be slightly less than actual max.
                             dDeviation,      // out: MaxDist(DropPt->SrfNormalLine, 3dCurvePt), Quality Measure: expected to be close to 0.0
                             sProjCurves));

      // draw, lift, and delete all drop result curves
      SER(my_test_drop_results(sContext,pNewBSS,sProjCurves,1));
#ifdef SM_GFX_CODE
      if(smGet_DoGraphics())
        { smgfx_SetLook(3,4, 0,0,1) ; if(p3DCurve) p3DCurve->Draw(NULL, TRUE) ; sm_GraphicsLoop() ;
          sm_GraphicsLoop() ; 
        }        
#endif // SM_GFX_CODE

      SM_ASSERT(p3DCurve != NULL) ; delete p3DCurve ; p3DCurve = NULL ;

    }

  // lift and drop a line UVTrimCurve to a doubly curved surface
  if (TRUE) 
    { // Test Lift/Drop of curves on surfaces which are C0
      SmBSplineSurface *pNewBSS   = my_create_crease_surf(sContext,SmPoint3d(8,0,0),sbCreases,1.0,0.5,0.0);
      SmObjDelete       sCU1(pNewBSS);
      SmExtent2d        sUVDomain = pNewBSS->GetNaturalUVDomain();
      SmBSplineCurve   *pLine     = my_create_line(sContext,
                                                   SmPoint3d(sUVDomain.Evaluate(0.9,0.5)),
                                                   SmPoint3d(sUVDomain.Evaluate(0.1,0.5)),
                                                   1,0,0);
      SmObjDelete       sCU2(pLine);
      SmBSplineCurve   *p3DCurve = NULL ;
      double            dMaxDist;

#ifdef SM_GFX_CODE
      if(smGet_DoGraphics())
        {
          SM_ASSERT_VALID(pNewBSS) ;
          SM_ASSERT_VALID(pLine) ;
          SmPoint3d sPt(.7,.5,.5);
          my_CenterSurface(pNewBSS,.65,&sPt);

          smgfx_Erase() ;
          smgfx_SetLook(1,2, 0,1,1) ; if(pNewBSS) pNewBSS->DrawUV(8,8,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ;
          smgfx_SetLook(3,4, 0,0,1) ; if(pLine) pNewBSS->DrawUVCurve(*pLine) ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 0,0,1) ; if(pLine) pNewBSS->DrawUVCurve(*pLine, TRUE) ; sm_GraphicsLoop() ;
          sm_GraphicsLoop() ;
        }        
#endif // SM_GFX_CODE

      SER(pNewBSS->LiftCurve(sContext,sUVDomain,
                            *pLine,pLine->GetNaturalInterval(),
                             1.0e-3,dMaxDist,p3DCurve));

      SmTArray<SmBSplineCurve*> sProjCurves;

      SER(pNewBSS->DropCurve(sContext,
                             pNewBSS->GetNaturalUVDomain(),
                            *p3DCurve,
                             p3DCurve->GetNaturalInterval(),
                             1.0e-2,
                             dMaxDistToSurf,   // out: MaxDist(DropPt, 3dCurvePt), found by sampling, may be slightly less than actual max.
                             dDeviation,       // out: MaxDist(DropPt->SrfNormalLine, 3dCurvePt), Quality Measure: expected to be close to 0.0
                             sProjCurves));

      // draw, lift, and delete all drop result curves
      SER(my_test_drop_results(sContext,pNewBSS,sProjCurves,1));
#ifdef SM_GFX_CODE
      if(smGet_DoGraphics())
        { smgfx_SetLook(3,4, 0,0,1) ; if(p3DCurve) p3DCurve->Draw(NULL, TRUE) ; sm_GraphicsLoop() ;
          sm_GraphicsLoop() ; 
        }        
#endif // SM_GFX_CODE

      SM_ASSERT(p3DCurve != NULL) ; delete p3DCurve ; p3DCurve = NULL ;
    }

  // lift and drop a isoParam UVTrimCurve to a doubly curved surface
  if (TRUE) 
    { // Test Lift/Drop of curves on surfaces which are C0
      SmBSplineSurface *pNewBSS   = my_create_crease_surf(sContext,SmPoint3d(8,0,0),sbCreases,1.0,0.5,0.0);
      SmObjDelete       sCU1(pNewBSS);
      SmExtent2d        sUVDomain = pNewBSS->GetNaturalUVDomain();
      SmBSplineCurve   *pLine     = my_create_line(sContext,
                                                   SmPoint3d(sUVDomain.Evaluate(0.37,0)),
                                                   SmPoint3d(sUVDomain.Evaluate(0.37,1.0)),
                                                   1,0,0);
      SmObjDelete       sCU2(pLine);
      SmBSplineCurve   *p3DCurve = NULL ;
      double            dMaxDist;

#ifdef SM_GFX_CODE
      if(smGet_DoGraphics())
        {
          SM_ASSERT_VALID(pNewBSS) ;
          SM_ASSERT_VALID(pLine) ;
          SmPoint3d sPt(.7,.5,.5);
          my_CenterSurface(pNewBSS,.65,&sPt);

          smgfx_Erase() ;
          smgfx_SetLook(1,2, 0,1,1) ; if(pNewBSS) pNewBSS->DrawUV(8,8,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ;
          smgfx_SetLook(3,4, 0,0,1) ; if(pLine) pNewBSS->DrawUVCurve(*pLine) ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 0,0,1) ; if(pLine) pNewBSS->DrawUVCurve(*pLine, TRUE) ; sm_GraphicsLoop() ;
          sm_GraphicsLoop() ;
        }        
#endif // SM_GFX_CODE

      SER(pNewBSS->LiftCurve(sContext,sUVDomain,*pLine,pLine->GetNaturalInterval(),
                             1.0e-3,dMaxDist,p3DCurve));

      SmTArray<SmBSplineCurve*> sProjCurves;

      SER(pNewBSS->DropCurve(sContext,
                             pNewBSS->GetNaturalUVDomain(),
                            *p3DCurve,
                             p3DCurve->GetNaturalInterval(),
                             1.0e-2,
                             dMaxDistToSurf,   // out: MaxDist(DropPt, 3dCurvePt), found by sampling, may be slightly less than actual max.
                             dDeviation,       // out: MaxDist(DropPt->SrfNormalLine, 3dCurvePt), Quality Measure: expected to be close to 0.0
                             sProjCurves));

      // draw, lift, and delete all drop result curves
      SER(my_test_drop_results(sContext,pNewBSS,sProjCurves,1));
#ifdef SM_GFX_CODE
      if(smGet_DoGraphics())
        { smgfx_SetLook(3,4, 0,0,1) ; if(p3DCurve) p3DCurve->Draw(NULL, TRUE) ; 
          sm_GraphicsLoop() ; 
      }        
#endif // SM_GFX_CODE

      SM_ASSERT(p3DCurve != NULL) ; delete p3DCurve ; p3DCurve = NULL ;
    }

  // lift and drop a partial isoParam UVTrimCurve to a doubly curved surface
  if (TRUE) 
    { // Test Lift/Drop of curves on surfaces which are C0
      SmBSplineSurface *pNewBSS   = my_create_crease_surf(sContext,SmPoint3d(8,0,0),sbCreases,1.0,0.5,0.0);
      SmObjDelete       sCU1(pNewBSS);
      SmExtent2d        sUVDomain = pNewBSS->GetNaturalUVDomain();
      SmBSplineCurve    *pLine    = my_create_line(sContext,
                                                   SmPoint3d(sUVDomain.Evaluate(0.75,0.8)),
                                                   SmPoint3d(sUVDomain.Evaluate(0.75,0.1)),
                                                   1,0,0);
      SmObjDelete        sCU2(pLine);
      SmBSplineCurve    *p3DCurve = NULL ;
      double             dMaxDist;


      SER(pNewBSS->LiftCurve(sContext,sUVDomain,*pLine,pLine->GetNaturalInterval(),
          1.0e-3,dMaxDist,p3DCurve));
      SmObjDelete sClean1( p3DCurve );

#ifdef SM_GFX_CODE
      if(smGet_DoGraphics())
        {
          SM_ASSERT_VALID(pNewBSS) ;
          SM_ASSERT_VALID(pLine) ;
          SmPoint3d sPt(.7,.5,.5);
          my_CenterSurface(pNewBSS,.65,&sPt);

          smgfx_Erase() ;
          smgfx_SetLook(1,2, 0,1,1) ; if(pNewBSS) pNewBSS->DrawUV(8,8,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ;
          smgfx_SetLook(3,4, 0,0,1) ; if(pLine) pNewBSS->DrawUVCurve(*pLine) ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 0,0,1) ; if(pLine) pNewBSS->DrawUVCurve(*pLine, TRUE) ; sm_GraphicsLoop() ;
          sm_GraphicsLoop() ;
        }        
#endif // SM_GFX_CODE

      SER(pNewBSS->LiftCurve(sContext,sUVDomain,*pLine,pLine->GetNaturalInterval(),
                             1.0e-3,dMaxDist,p3DCurve));

      SmTArray<SmBSplineCurve*> sProjCurves;

      SER(pNewBSS->DropCurve(sContext,
                             pNewBSS->GetNaturalUVDomain(),
                            *p3DCurve,
                             p3DCurve->GetNaturalInterval(),
                             1.0e-2,
                             dMaxDistToSurf,   // out: MaxDist(DropPt, 3dCurvePt), found by sampling, may be slightly less than actual max.
                             dDeviation,       // out: MaxDist(DropPt->SrfNormalLine, 3dCurvePt), Quality Measure: expected to be close to 0.0
                             sProjCurves));

      // draw, lift, and delete all drop result curves
      SER(my_test_drop_results(sContext,pNewBSS,sProjCurves,1));
#ifdef SM_GFX_CODE
      if(smGet_DoGraphics())
        { smgfx_SetLook(3,4, 0,0,1) ; if(p3DCurve) p3DCurve->Draw(NULL, TRUE) ;
          sm_GraphicsLoop() ; 
        }        
#endif // SM_GFX_CODE

      SM_ASSERT(p3DCurve != NULL) ; delete p3DCurve ; p3DCurve = NULL ;
    }

  //
  if (TRUE) 
    { // Test Lift/Drop of curves on surfaces which are C0
      SmBSplineSurface *pNewBSS   = my_create_crease_surf(sContext,SmPoint3d(8,0,0),sbCreases,1.0,0.5,0.0);
      SmObjDelete       sCU1(pNewBSS);
      SmExtent2d        sUVDomain = pNewBSS->GetNaturalUVDomain();
      SmBSplineCurve   *pLine     = my_create_line(sContext,
                                                   SmPoint3d(sUVDomain.Evaluate(0.47,0.1)),
                                                   SmPoint3d(sUVDomain.Evaluate(0.47,0.8)),
                                                   1,0,0);
      SmObjDelete       sCU2(pLine);
      SmBSplineCurve   *p3DCurve = NULL ;
      double            dMaxDist;

#ifdef SM_GFX_CODE
      if(smGet_DoGraphics())
        {
          SM_ASSERT_VALID(pNewBSS) ;
          SM_ASSERT_VALID(pLine) ;
          SmPoint3d sPt(.7,.5,.5);
          my_CenterSurface(pNewBSS,.65,&sPt);

          smgfx_Erase() ;
          smgfx_SetLook(1,2, 0,1,1) ; if(pNewBSS) pNewBSS->DrawUV(8,8,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ;
          smgfx_SetLook(3,4, 0,0,1) ; if(pLine) pNewBSS->DrawUVCurve(*pLine) ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 0,0,1) ; if(pLine) pNewBSS->DrawUVCurve(*pLine, TRUE) ; sm_GraphicsLoop() ;
          sm_GraphicsLoop() ;
        }        
#endif // SM_GFX_CODE

      SER(pNewBSS->LiftCurve(sContext,sUVDomain,*pLine,pLine->GetNaturalInterval(),
                             1.0e-3,dMaxDist,p3DCurve));

      SmTArray<SmBSplineCurve*> sProjCurves;
   
      SER(pNewBSS->DropCurve(sContext,
                             pNewBSS->GetNaturalUVDomain(),
                            *p3DCurve,
                             p3DCurve->GetNaturalInterval(),
                             1.0e-2,
                             dMaxDistToSurf,   // out: MaxDist(DropPt, 3dCurvePt), found by sampling, may be slightly less than actual max.
                             dDeviation,       // out: MaxDist(DropPt->SrfNormalLine, 3dCurvePt), Quality Measure: expected to be close to 0.0
                             sProjCurves));

      // draw, lift, and delete all drop result curves
      SER(my_test_drop_results(sContext,pNewBSS,sProjCurves,1));
#ifdef SM_GFX_CODE
      if(smGet_DoGraphics())
        { smgfx_SetLook(3,4, 0,0,1) ; if(p3DCurve) p3DCurve->Draw(NULL, TRUE) ;
          sm_GraphicsLoop() ; 
        }        
#endif // SM_GFX_CODE

      SM_ASSERT(p3DCurve != NULL) ; delete p3DCurve ; p3DCurve = NULL ;
    }


//    pPool->Free();
//    delete pPool; pPool = NULL ;
  return SM_SUCCESS;

} // end my_test_drop_curve

/***********************************************************************
PURPOSE ---

USAGE NOTES ---
***********************************************************************/
PT_EXPORT SmStatus my_test_ss_nurb
 (void)
{
//    SmContext sContext(pPool);
    SmContext sContext;

    SmTArray<SmPoint3d> sPnts;
    sPnts.Add(SmPoint3d(0,0,0));
    sPnts.Add(SmPoint3d(1,0.1,0.4));
    sPnts.Add(SmPoint3d(2,0.2,0.3));
    sPnts.Add(SmPoint3d(3,0.3,0));
    sPnts.Add(SmPoint3d(0,1,0.2));
    sPnts.Add(SmPoint3d(1,1,1.5));
    sPnts.Add(SmPoint3d(2,1,1.6));
    sPnts.Add(SmPoint3d(3,1,0.2));
    sPnts.Add(SmPoint3d(0,2,0.3));
    sPnts.Add(SmPoint3d(1,2,1.6));
    sPnts.Add(SmPoint3d(2,2,1.5));
    sPnts.Add(SmPoint3d(3,2,0.4));
    sPnts.Add(SmPoint3d(0,3,0.4));
    sPnts.Add(SmPoint3d(1,3,1.2));
    sPnts.Add(SmPoint3d(2,3,1.1));
    sPnts.Add(SmPoint3d(3,3,0.7));
    sPnts.Add(SmPoint3d(0,4,0.4));
    sPnts.Add(SmPoint3d(1,4,0.2));
    sPnts.Add(SmPoint3d(2,4,0.1));
    sPnts.Add(SmPoint3d(3,4,0.5));

    SmBSplineSurface *pNewBSS = my_create_nurb_surf(sContext,SmPoint3d(5,0,0),sPnts,5,4,3,3,1.0,0.5,0);

    SmObjDelete sCU1(pNewBSS);

    SmTArray<SmPoint3d> sPnts2;
    sPnts2.Add(SmPoint3d(0,0,0.1));
    sPnts2.Add(SmPoint3d(1,0.1,0.3));
    sPnts2.Add(SmPoint3d(2,0.2,0.4));
    sPnts2.Add(SmPoint3d(3,0.3,0));
    sPnts2.Add(SmPoint3d(0,1,0.3));
    sPnts2.Add(SmPoint3d(1,1,0.6));
    sPnts2.Add(SmPoint3d(2,1,-1.5));
    sPnts2.Add(SmPoint3d(3,1,-1.2));
    sPnts2.Add(SmPoint3d(0,2,0.4));
    sPnts2.Add(SmPoint3d(1,2,0.5));
    sPnts2.Add(SmPoint3d(2,2,-1.6));
    sPnts2.Add(SmPoint3d(3,2,-1.3));
    sPnts2.Add(SmPoint3d(0,3,0.7));
    sPnts2.Add(SmPoint3d(1,3,-1.1));
    sPnts2.Add(SmPoint3d(2,3,-1.2));
    sPnts2.Add(SmPoint3d(3,3,0.4));
    sPnts2.Add(SmPoint3d(0,4,0.7));
    sPnts2.Add(SmPoint3d(1,4,0.1));
    sPnts2.Add(SmPoint3d(2,4,0.2));
    sPnts2.Add(SmPoint3d(3,4,0.6));
    SmBSplineSurface *pNewBSS2 = my_create_nurb_surf(sContext,SmPoint3d(5,0,3),sPnts2,5,4,3,3,0,0,0);
    SmObjDelete sCU2(pNewBSS2);


    ULONG lCount;
#ifdef SM_GFX_CODE
    smgfx_SetColor(1.0,0.0,0.0);
#endif // SM_GFX_CODE
    SER(my_test_ss_solve(*pNewBSS,*pNewBSS2,SM_SO_MINIMIZE,lCount));
#ifdef SM_GFX_CODE
    smgfx_SetColor(0,0,1);
#endif // SM_GFX_CODE
    SER(my_test_ss_solve(*pNewBSS,*pNewBSS2,SM_SO_MAXIMIZE,lCount));
#ifdef SM_GFX_CODE
    smgfx_SetColor(1.0,0.0,1.0);
#endif // SM_GFX_CODE
    SER(my_test_ss_solve(*pNewBSS,*pNewBSS2,SM_SO_NORMALIZE,lCount));
    SM_ASSERT(lCount == 1);

    SmBSplineSurface *pNewBSS3 = my_create_nurb_surf(sContext,SmPoint3d(0,0,0),sPnts,5,4,3,3,1.0,0.5,0);
    SmObjDelete sCU3(pNewBSS3);
    SmBSplineSurface *pNewBSS4 = my_create_nurb_surf(sContext,SmPoint3d(0,0,1),sPnts2,5,4,3,3,0,0,0);
    SmObjDelete sCU4(pNewBSS4);
#ifdef SM_GFX_CODE
    smgfx_SetColor(1.0,0.0,0.0);
#endif // SM_GFX_CODE
    SER(my_test_ss_solve(*pNewBSS3,*pNewBSS4,SM_SO_MINIMIZE,lCount));
    SM_ASSERT(lCount > 50);

    return SM_SUCCESS;

} // end my_test_ss_nurb

/**************************************************************
PURPOSE ---  analytic surface intersection test case

USAGE NOTES ---
  o. test1: sequence of planes against an array of lines
  o. test2: sequence of cylinders against an array of line
**************************************************************/
PT_EXPORT SmStatus my_test_csi_analy(void)
{
//    SmContext sContext(pPool);
    SmContext sContext;
    ULONG lNumFound;

    if (FALSE)
      {
        SmContext context;
        SmBSplineSurface *surface = NULL ;
        SER (SmBSplineSurface::ReadFromFile (context, _T("Lift_surface000.sm"),
            surface));
        SmBSplineCurve *curve = NULL ;
        SER (SmBSplineCurve::ReadFromFile (context, _T("Lift_curve000.sm"),
            curve));

        SmApproxTol3d tolerance = 0.001;
        SmTArray<SmCurve*> projected_curves;
        SER (surface->CreateParallelProjectionCurves (context,
                                                      surface->GetNaturalUVDomain (),
                                                      *curve,
                                                      SmVector3d (0.0, 0.0, 1.0),
                                                      0, 
                                                      &tolerance,
                                                      0,
                                                      &projected_curves, 
                                                      0));

        printf ("Completed successfully\n");

        return SM_SUCCESS;
      } // end if FALSE

    if (FALSE)
      {
        SmBSplineSurface *pBSS = NULL ;
        SmStatus res;

        ULONG lUDegree, lVDegree;
        SmTArray<SmPoint3d> rControlPointsList;
        SmTArray<ULONG> crUMultiplicities;
        SmTArray<ULONG> crVMultiplicities;
        SmTArray<double> crUKnots;
        SmTArray<double> crVKnots;
        SmTArray<double> cOptWeights;
        SmExtent2d cOptUVDomain;


        //****************

        lUDegree = 3; lVDegree = 2;     //  Num U Points = 5  Num V Points = 7

        // SmTArray<double> crUKnots;    SmTArray<ULONG> crUMultiplicities;
        crUKnots.Add(0.00000000000000);  crUMultiplicities.Add(4); // [0]
        crUKnots.Add(1.00000000000000);  crUMultiplicities.Add(1); // [1]
        crUKnots.Add(2.00000000000000);  crUMultiplicities.Add(4); // [2]

        // SmTArray<double> crVKnots;    SmTArray<ULONG> crVMultiplicities;
        crVKnots.Add(0.00000000000000);  crVMultiplicities.Add(3); // [0]
        crVKnots.Add(1.00000000000000);  crVMultiplicities.Add(1); // [1]
        crVKnots.Add(2.00000000000000);  crVMultiplicities.Add(1); // [2]
        crVKnots.Add(3.00000000000000);  crVMultiplicities.Add(1); // [3]
        crVKnots.Add(4.00000000000000);  crVMultiplicities.Add(1); // [4]
        crVKnots.Add(5.00000000000000);  crVMultiplicities.Add(3); // [5]

        // ***** Control Points ******
        rControlPointsList.Add(SmPoint3d(-9.99999999999999, -90.00000000000001, -40.00000000000000));       //   [0][0] =>> 0
        rControlPointsList.Add(SmPoint3d(-15.00993828742204, -90.00000000000000, -37.49503085628898));       //   [0][1] =>> 1
        rControlPointsList.Add(SmPoint3d(-20.89918488675400, -90.00000000000000, -29.46066919081585));       //   [0][2] =>> 2
        rControlPointsList.Add(SmPoint3d(-29.86166135600070, -90.00000000000000, -24.45419962835159));       //   [0][3] =>> 3
        rControlPointsList.Add(SmPoint3d(-41.04540877725864, -90.00000000000000, -21.58500943904096));       //   [0][4] =>> 4
        rControlPointsList.Add(SmPoint3d(-54.08153368920120, -90.00000000000000, -19.79899179084966));       //   [0][5] =>> 5
        rControlPointsList.Add(SmPoint3d(-62.62953406581700, -90.00000000000000, -23.21819194149598));       //   [0][6] =>> 6
        rControlPointsList.Add(SmPoint3d(-9.99999999999999, -20.00000000000001, -50.00000000000001));       //   [1][0] =>> 7
        rControlPointsList.Add(SmPoint3d(-14.17192594490645, -20.00000000000001, -47.91403702754678));       //   [1][1] =>> 8
        rControlPointsList.Add(SmPoint3d(-18.11902186410478, -19.99999999999999, -39.11280302897903));       //   [1][2] =>> 9
        rControlPointsList.Add(SmPoint3d(-25.35399362720083, -20.00000000000004, -33.42503955402192));       //   [1][3] =>> 10
        rControlPointsList.Add(SmPoint3d(-35.09449053271035, -20.00000000000002, -29.66201132684917));       //   [1][4] =>> 11
        rControlPointsList.Add(SmPoint3d(-47.19784042704145, -20.00000000000001, -26.67879014901961));       //   [1][5] =>> 12
        rControlPointsList.Add(SmPoint3d(-55.15544087898040, -20.00000000000001, -29.86183032979519));       //   [1][6] =>> 13
        rControlPointsList.Add(SmPoint3d(-9.99999999999999, -10.00000000000001, -20.00000000000001));       //   [2][0] =>> 14
        rControlPointsList.Add(SmPoint3d(-16.68596297245324, -10.00000000000001, -16.65701851377339));       //   [2][1] =>> 15
        rControlPointsList.Add(SmPoint3d(-26.45951093205240, -10.00000000000001, -10.15640151448953));       //   [2][2] =>> 16
        rControlPointsList.Add(SmPoint3d(-38.87699681360041, -10.00000000000001, -6.51251977701096));       //   [2][3] =>> 17
        rControlPointsList.Add(SmPoint3d(-52.94724526635518, -10.00000000000001, -5.43100566342460));       //   [2][4] =>> 18
        rControlPointsList.Add(SmPoint3d(-67.84892021352071, -10.00000000000001, -6.03939507450981));       //   [2][5] =>> 19
        rControlPointsList.Add(SmPoint3d(-77.57772043949019, -10.00000000000001, -9.93091516489760));       //   [2][6] =>> 20
        rControlPointsList.Add(SmPoint3d(-10.00000000000001, -39.99999999999999, 39.99999999999999));       //   [3][0] =>> 21
        rControlPointsList.Add(SmPoint3d(-21.71403702754681, -39.99999999999999, 45.85701851377338));       //   [3][1] =>> 22
        rControlPointsList.Add(SmPoint3d(-43.14048906794761, -39.99999999999999, 47.75640151448948));       //   [3][2] =>> 23
        rControlPointsList.Add(SmPoint3d(-65.92300318639960, -40.00000000000000, 47.31251977701093));       //   [3][3] =>> 24
        rControlPointsList.Add(SmPoint3d(-88.65275473364481, -40.00000000000000, 43.03100566342455));       //   [3][4] =>> 25
        rControlPointsList.Add(SmPoint3d(-109.15107978647926, -39.99999999999999, 35.23939507450980));       //   [3][5] =>> 26
        rControlPointsList.Add(SmPoint3d(-122.42227956050978, -39.99999999999999, 29.93091516489760));       //   [3][6] =>> 27
        rControlPointsList.Add(SmPoint3d(-10.00000000000000, -90.00000000000000, 10.00000000000000));       //   [4][0] =>> 28
        rControlPointsList.Add(SmPoint3d(-19.20000000000002, -90.00000000000000, 14.60000000000001));       //   [4][1] =>> 29
        rControlPointsList.Add(SmPoint3d(-34.80000000000001, -90.00000000000000, 18.80000000000000));       //   [4][2] =>> 30
        rControlPointsList.Add(SmPoint3d(-52.40000000000000, -90.00000000000000, 20.40000000000000));       //   [4][3] =>> 31
        rControlPointsList.Add(SmPoint3d(-70.80000000000000, -90.00000000000000, 18.80000000000000));       //   [4][4] =>> 32
        rControlPointsList.Add(SmPoint3d(-88.50000000000000, -90.00000000000000, 14.60000000000000));       //   [4][5] =>> 33
        rControlPointsList.Add(SmPoint3d(-99.99999999999999, -90.00000000000000, 10.00000000000001));       //   [4][6] =>> 34
        //****************

        res = SmBSplineSurface::CreateCanonical(sContext,
            lUDegree, lVDegree,
            rControlPointsList,
            SM_SF_UNSPECIFIED,
            crUMultiplicities, crVMultiplicities,
            crUKnots, crVKnots,
            SM_KT_UNSPECIFIED,
            NULL, NULL, pBSS);
        if (res!=SM_SUCCESS) return SM_ERR;

        SmPoint3d crViewVector(0.0, 0.0, 1.0);
        SmApproxTol3d cpdOpt3DApproximationTol = 0.001;
        double cpdOptAngleTolRadians = 0.1;
        SmTArray<SmCurve*> s3DSilhouetteCurves;
        SmObjsDelete<SmCurve*> sCleanup_rCurves(&s3DSilhouetteCurves);

        // Create the SilhouetteCurves
        // ---------------------------------
        res = pBSS->CreateSilhouetteCurves(sContext,
                                           pBSS->GetNaturalUVDomain(),    
                                           crViewVector, 
                                           FALSE,
                                           &cpdOpt3DApproximationTol, 
                                           &cpdOptAngleTolRadians, 
                                           &s3DSilhouetteCurves, 
                                           NULL);
        if (res!=SM_SUCCESS) return SM_ERR;

        for (ULONG i=0; i<s3DSilhouetteCurves.GetSize(); i++)
          {
#ifdef SM_GFX_CODE
            SmBSplineCurve * pSil = SM_CAST_PTR(SmBSplineCurve,s3DSilhouetteCurves[i]);

            if ( smGet_DoGraphics() )
            {
                smgfx_SetColor( 1, 0, 0 );
                pSil->DrawWithKnots();

                smgfx_SetColor( 0, 0, 1 );
                pBSS->DrawUV( 10, 10 );
            }
#endif // SM_GFX_CODE
          }
        SM_ASSERT(pBSS != NULL) ; delete pBSS ; pBSS = NULL ;
        return SM_SUCCESS;
      } // end if FALSE


    // intersect sequence of planes with several line curves
    {
        SmBSplineSurface *pPlane = NULL ;
        pPlane = my_create_plane(sContext,
                                 SmPoint3d(0,0,0),
                                 SmPoint3d(0,0,0),
                                 SmPoint3d(2,0,0),
                                 SmPoint3d(0,2,0),
                                 0,0,0);
        SmObjDelete sCU1(pPlane);
        SER(my_test_surface_curve_intersect(*pPlane,pPlane->GetNaturalUVDomain(),5,5,lNumFound));
        SM_ASSERT(lNumFound == 25);

        pPlane = my_create_plane(sContext,SmPoint3d(0,-5,0),
                                 SmPoint3d(0,0,0),
                                 SmPoint3d(2,0,2),
                                 SmPoint3d(0,2,0),
                                 0,0,0);
        SmObjDelete sCU2(pPlane);
        SER(my_test_surface_curve_intersect(*pPlane,pPlane->GetNaturalUVDomain(),5,5,lNumFound));
        SM_ASSERT(lNumFound == 25);

        pPlane = my_create_plane(sContext,
                                 SmPoint3d(9,0,0),
                                 SmPoint3d(0,0,0),
                                 SmPoint3d(2,0,2),
                                 SmPoint3d(0,2,2),
                                 0,0,0);
        SmObjDelete sCU3(pPlane);
        SER(my_test_surface_curve_intersect(*pPlane,pPlane->GetNaturalUVDomain(),5,5,lNumFound));
        SM_ASSERT(lNumFound == 25);

        pPlane = my_create_plane(sContext,
                                 SmPoint3d(14,0,0),
                                 SmPoint3d(0,0,0),
                                 SmPoint3d(0,0,2),
                                 SmPoint3d(0,2,0),
                                 0,0,0);
        SmObjDelete sCU4(pPlane);
        SER(my_test_surface_curve_intersect(*pPlane,pPlane->GetNaturalUVDomain(),5,5,lNumFound));
        SM_ASSERT(lNumFound == 5);
    } // end intersect sequence of planes with several line curves

    // intersect a sequence of cylinders with an array of lines
    {
        SmBSplineSurface *pCylBSS = NULL ;
        SmAxis2Placement sA2P;

        // cyl 1
        sA2P.SetCanonical(SmPoint3d(-5,0,0),
                          SmVector3d(1,0,0),
                          SmVector3d(0,0,1));
        SER(SmBSplineSurface::CreateConePatch(sContext, sA2P,
                                              1.0, 1.0, 0.0,
                                              360.0, 1.0,
                                              SM_CO_QUADRATIC,
                                              pCylBSS));
        SmObjDelete sCU1(pCylBSS);
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics())
          {
            SM_ASSERT_VALID(pCylBSS) ;
            smgfx_Erase() ;
            smgfx_SetLook(1,2, 0,1,1); pCylBSS->DrawUV(2,2); sm_GraphicsLoop() ;
            sm_GraphicsLoop() ;
          }
#endif // SM_GFX_CODE
        SER(my_test_surface_curve_intersect(*pCylBSS,pCylBSS->GetNaturalUVDomain(),5,5,lNumFound));
        SM_ASSERT(lNumFound == 45);

        // cyl 2
        sA2P.SetCanonical(SmPoint3d(-5,5,0),
                          SmVector3d(1,0,0),
                          SmVector3d(0,0,1));
        SER(SmBSplineSurface::CreateConePatch(sContext, sA2P,
                                              1.0,1.0,0.0,
                                              180.0, 1.0,
                                              SM_CO_QUADRATIC,
                                              pCylBSS));
        SmObjDelete sCU2(pCylBSS);
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
            SM_ASSERT_VALID(pCylBSS) ;
            smgfx_Erase() ;
            smgfx_SetLook(1,2, 0,1,1); pCylBSS->DrawUV(2,2); sm_GraphicsLoop() ;
            sm_GraphicsLoop() ;
          }
#endif // SM_GFX_CODE
        SER(my_test_surface_curve_intersect(*pCylBSS,pCylBSS->GetNaturalUVDomain(),5,5,lNumFound));
        SM_ASSERT(lNumFound == 25);

        // cyl 3
        sA2P.SetCanonical(SmPoint3d(5,0,0),
                          SmVector3d(1,0,0),
                          SmVector3d(0,1,0));
        SER(SmBSplineSurface::CreateConePatch(sContext, sA2P,
                                              1.0,0.0,0.0,
                                              360.0, 1.0,
                                              SM_CO_QUADRATIC,
                                              pCylBSS));
        SmObjDelete sCU3(pCylBSS);
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics())  {
            SM_ASSERT_VALID(pCylBSS) ;
            smgfx_Erase() ;
            smgfx_SetLook(1,2, 0,1,1); pCylBSS->DrawUV(2,2); sm_GraphicsLoop() ;
            sm_GraphicsLoop() ;
          }
#endif // SM_GFX_CODE
        SER(my_test_surface_curve_intersect(*pCylBSS,pCylBSS->GetNaturalUVDomain(),10,10,lNumFound));
        SM_ASSERT(lNumFound == 60);

        // cyl 4
        sA2P.SetCanonical(SmPoint3d(5,-5,0),
                          SmVector3d(1,0,0),
                          SmVector3d(0,1,0));
        SER(SmBSplineSurface::CreateConePatch(sContext, sA2P,
                                              0.0,1.0,0.0,
                                              270.0,1.0,
                                              SM_CO_QUADRATIC,
                                              pCylBSS));
        SmObjDelete sCU4(pCylBSS);
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
            SM_ASSERT_VALID(pCylBSS) ;
            smgfx_Erase() ;
            smgfx_SetLook(1,2, 0,1,1); pCylBSS->DrawUV(2,2); sm_GraphicsLoop() ;
            sm_GraphicsLoop() ;
          }
#endif // SM_GFX_CODE
        SER(my_test_surface_curve_intersect(*pCylBSS,pCylBSS->GetNaturalUVDomain(),10,10,lNumFound));
        SM_ASSERT(lNumFound == 45);

        // cyl 5
        sA2P.SetCanonical(SmPoint3d(0,5,0),
                          SmVector3d(1,0,0),
                          SmVector3d(0,0,1));
        SER(SmBSplineSurface::CreateConePatch(sContext, sA2P,
                                              1.0,1.0,0.0,
                                              360.0,1.0,
                                              SM_CO_QUINTIC,
                                              pCylBSS));
        SmObjDelete sCU5(pCylBSS);
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
            SM_ASSERT_VALID(pCylBSS) ;
            smgfx_Erase() ;
            smgfx_SetLook(1,2, 0,1,1); pCylBSS->DrawUV(2,2); sm_GraphicsLoop() ;
            sm_GraphicsLoop() ;
          }
#endif // SM_GFX_CODE
        SER(my_test_surface_curve_intersect(*pCylBSS,pCylBSS->GetNaturalUVDomain(),5,5,lNumFound));
        SM_ASSERT(lNumFound == 25);

        // cyl 6
        sA2P.SetCanonical(SmPoint3d(5,5,0),
                          SmVector3d(1,0,0),
                          SmVector3d(0,0,1));
        SER(SmBSplineSurface::CreateConePatch(sContext, sA2P,
                                              0.0,1.0,0.0,
                                              360.0,1.0,
                                              SM_CO_QUINTIC,
                                              pCylBSS));
        SmObjDelete sCU6(pCylBSS);
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
            SM_ASSERT_VALID(pCylBSS) ;
            smgfx_Erase() ;
            smgfx_SetLook(1,2, 0,1,1); pCylBSS->DrawUV(2,2); sm_GraphicsLoop() ;
            sm_GraphicsLoop() ;
          }
#endif // SM_GFX_CODE
        SER(my_test_surface_curve_intersect(*pCylBSS,pCylBSS->GetNaturalUVDomain(),5,5,lNumFound));
        SM_ASSERT(lNumFound == 13); // was 12 [bd]: same as the next test: seam at a singularity.

        // cyl 7
        sA2P.SetCanonical(SmPoint3d(-5,-5,0),
                          SmVector3d(1,0,0),
                          SmVector3d(0,0,1));
        SER(SmBSplineSurface::CreateConePatch(sContext, sA2P,
                                              1.0,0.0,0.0,
                                              360.0,1.0,
                                              SM_CO_QUINTIC,
                                              pCylBSS));
        SmObjDelete sCU7(pCylBSS);
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
            SM_ASSERT_VALID(pCylBSS) ;
            smgfx_Erase() ;
            smgfx_SetLook(1,2, 0,1,1); pCylBSS->DrawUV(2,2); sm_GraphicsLoop() ;
            sm_GraphicsLoop() ;
          }
#endif // SM_GFX_CODE
        SER(my_test_surface_curve_intersect(*pCylBSS,pCylBSS->GetNaturalUVDomain(),5,5,lNumFound));
        SM_ASSERT(lNumFound == 13);   // was 12 GWC: 13 is correct due to returning multiple solutions on a seam rule
        }

//    pPool->Free();
//    delete pPool; pPool = NULL ;

    return SM_SUCCESS;

} // end my_test_csi_analy

/***********************************************************************
PURPOSE ---

USAGE NOTES ---
***********************************************************************/
PT_EXPORT SmStatus my_test_csi_nurb(void)
{
//    SmContext sContext(pPool);
    SmContext sContext;

    if (FALSE) { // Test domain bounded surfaces
    SmTArray<SmPoint3d> sPnts;
    my_load_surf1_pts(sPnts);
    SmBSplineSurface *pNewBSS = my_create_crease_surf(sContext,SmPoint3d(8,0,0),sbCreases,1.0,0.5,0.0);
    SmObjDelete sCU1(pNewBSS);


    ULONG lNumFound;
    SmExtent2d sUVDom = pNewBSS->GetNaturalUVDomain();
    sUVDom.SetMinMax(sUVDom.Evaluate(0.1,0.1),sUVDom.Evaluate(0.9,0.9));
    SER(my_test_surface_curve_intersect(*pNewBSS,sUVDom,10,10,lNumFound));
    SM_ASSERT(lNumFound == 49);
    }

    {
    SmTArray<SmPoint3d> sPnts;
    my_load_surf1_pts(sPnts);
    SmBSplineSurface *pNewBSS = NULL ;
    pNewBSS = my_create_crease_surf(sContext,SmPoint3d(0,0,0),sbCreases,1.0,0.5,0.0);
    SmObjDelete sCU1(pNewBSS);


    ULONG lNumFound;
    SER(my_test_surface_curve_intersect(*pNewBSS,pNewBSS->GetNaturalUVDomain(),10,10,lNumFound));
    SM_ASSERT(lNumFound == 92);

    SmAxis2Placement sA2P2;
    sA2P2.SetCanonical(SmPoint3d(0,10,-1),SmVector3d(1,0,0),SmVector3d(0,0,1));
    pNewBSS->Transform(sA2P2);
#ifdef SM_GFX_CODE
    if ( smGet_DoGraphics() )
    {
        smgfx_SetColor( 1.0, 0.5, 0.0 );
        pNewBSS->DrawUV( 3, 3 );
    }
#endif // SM_GFX_CODE
    SER(my_test_surface_curve_intersect(*pNewBSS,pNewBSS->GetNaturalUVDomain(),10,10,lNumFound));
    SM_ASSERT(lNumFound == 85); // gwc: used to be 84 - upon inpsection 85 looks ok

    sA2P2.SetCanonical(SmPoint3d(-13,1,0),SmVector3d(0,1,0),SmVector3d(0,0,1));
    pNewBSS->Transform(sA2P2);
#ifdef SM_GFX_CODE
    if ( smGet_DoGraphics() )
    {
        smgfx_SetColor( 1.0, 0.5, 0.0 );
        pNewBSS->DrawUV( 3, 3 );
    }
#endif // SM_GFX_CODE
    SER(my_test_surface_curve_intersect(*pNewBSS,pNewBSS->GetNaturalUVDomain(),10,10,lNumFound));
    SM_ASSERT(lNumFound == 92);

    pNewBSS = my_create_crease_surf(sContext,SmPoint3d(-6,0,0),sbCreases,0.0,0.0,0.0);
    SmObjDelete sCU2(pNewBSS);
#ifdef SM_GFX_CODE
    smgfx_SetLineWidth(2.0);
#endif // SM_GFX_CODE
    SmBSplineCurve *pBSC = my_create_3dcrv(sContext,SmPoint3d(-3,0,0),0.2,0,0.3);
    SmObjDelete sCU3(pBSC);
    my_test_GlobalCSI(*pNewBSS,*pBSC,SM_ZONE_TOL_3D,lNumFound);
    SM_ASSERT(lNumFound == 7);
    pBSC = my_create_3dcrv(sContext,SmPoint3d(-3.5,0,0),0.2,0,0.3);
    SmObjDelete sCU4(pBSC);
    my_test_GlobalCSI(*pNewBSS,*pBSC,SM_ZONE_TOL_3D,lNumFound);
    SM_ASSERT(lNumFound == 7);
    pBSC = my_create_3dcrv(sContext,SmPoint3d(-4,0,0),0.2,0,0.3);
    SmObjDelete sCU5(pBSC);
    my_test_GlobalCSI(*pNewBSS,*pBSC,SM_ZONE_TOL_3D,lNumFound);
    SM_ASSERT(lNumFound == 6); // used to be 8: upon inspection 6 is correct
    pBSC = my_create_3dcrv(sContext,SmPoint3d(-4.5,0,0),0.2,0,0.3);
    SmObjDelete sCU6(pBSC);
    my_test_GlobalCSI(*pNewBSS,*pBSC,SM_ZONE_TOL_3D,lNumFound);
    SM_ASSERT(lNumFound == 6);
    pBSC = my_create_3dcrv(sContext,SmPoint3d(-5,0,0),0.2,0,0.3);
    SmObjDelete sCU7(pBSC);
    my_test_GlobalCSI(*pNewBSS,*pBSC,SM_ZONE_TOL_3D,lNumFound);
    SM_ASSERT(lNumFound == 6); // used to be 8: upon inspection 6 is correct
    pBSC = my_create_3dcrv(sContext,SmPoint3d(-5.5,0,0),0.2,0,0.3);
    SmObjDelete sCU8(pBSC);
    my_test_GlobalCSI(*pNewBSS,*pBSC,SM_ZONE_TOL_3D,lNumFound);
    SM_ASSERT(lNumFound == 8);
    pBSC = my_create_3dcrv(sContext,SmPoint3d(-6,0,0),0.2,0,0.3);
    SmObjDelete sCU9(pBSC);
    my_test_GlobalCSI(*pNewBSS,*pBSC,SM_ZONE_TOL_3D,lNumFound);
    SM_ASSERT(lNumFound == 8);
    pBSC = my_create_3dcrv(sContext,SmPoint3d(-2,0,0),0.2,0,0.3);
    SmObjDelete sCU10(pBSC);
    my_test_GlobalCSI(*pNewBSS,*pBSC,SM_ZONE_TOL_3D,lNumFound);
    SM_ASSERT(lNumFound == 4);
    pBSC = my_create_3dcrv(sContext,SmPoint3d(-2.5,0,0),0.2,0,0.3);
    SmObjDelete sCU11(pBSC);
    my_test_GlobalCSI(*pNewBSS,*pBSC,SM_ZONE_TOL_3D,lNumFound);
    SM_ASSERT(lNumFound == 6);
    }
//    pPool->Free();
//    delete pPool; pPool = NULL ;

    return SM_SUCCESS;

} // end my_test_csi_nurb

/***********************************************************************
PURPOSE ---

USAGE NOTES ---
***********************************************************************/
PT_EXPORT SmStatus my_test_surf_methods(void)
{
    SmContext sContext;

    {
       SmBSplineSurface *pCylBSS = NULL ;
       SmAxis2Placement sA2P;
       sA2P.SetCanonical(SmPoint3d(0,0,0),SmVector3d(1,0,0),SmVector3d(0,1,0));
       SmExtent3d sBBox(SmPoint3d(-2,-2,-2),SmPoint3d(2,2,2));
       SER(SmBSplineSurface::CreateConeThroughBox(sContext,
           sA2P,1.0,45.0,0.0,180.0,SM_CO_QUINTIC,sBBox,pCylBSS));
       SmObjDelete sCleanupSur(pCylBSS);
    }

   {
       SmBSplineSurface *pCylBSS = NULL ;
       SmAxis2Placement sA2P;
       sA2P.SetCanonical(SmPoint3d(0,0,0),SmVector3d(1,0,0),SmVector3d(0,1,0));
       SER(SmBSplineSurface::CreateConePatch(sContext,
           sA2P,1.0,1.0,0.0,360.0,1.0,SM_CO_QUINTIC,pCylBSS));
       SmObjDelete sCleanupSur(pCylBSS);

       if (!pCylBSS->IsRational()) SER(SM_ERR);
       double dUKnots[100];
       double dVKnots[100];
       double dPoints[300];
       ULONG lUDeg = pCylBSS->GetDegree(SM_SP_U);
       ULONG lUNumKnots = pCylBSS->GetNumberNaturalKnots(SM_SP_U);
       ULONG lUNumCpts = pCylBSS->GetNumberControlPoints(SM_SP_U);
       SER(pCylBSS->GetKnotsExpert(SM_SP_U,0,lUNumKnots-1,dUKnots));

       ULONG lVDeg = pCylBSS->GetDegree(SM_SP_V);
       ULONG lVNumKnots = pCylBSS->GetNumberNaturalKnots(SM_SP_V);
       ULONG lVNumCpts = pCylBSS->GetNumberControlPoints(SM_SP_V);
       SER(pCylBSS->GetKnotsExpert(SM_SP_V,0,lVNumKnots-1,dVKnots));
       SER(pCylBSS->GetControlPointsExpert(SM_CP_HOMOGENEOUS_RATIONAL,0,lUNumCpts-1,0,lVNumCpts-1,
           4*lVNumCpts,4,dPoints));
       SmBSplineSurface sTest;
       SER(sTest.SetExpert(lUDeg,lVDeg,SM_SF_UNSPECIFIED,
           SM_EK_CLAMPPED,lUNumKnots,dUKnots,lVNumKnots,dVKnots,
           SM_CP_HOMOGENEOUS_RATIONAL,
           4*lVNumCpts,4,dPoints));
       pCylBSS->Dump(TRUE);
       sTest.Dump(TRUE);
    }


    return SM_SUCCESS;

} // end my_test_surf_methods



/***********************************************************************
PURPOSE ---

USAGE NOTES ---
***********************************************************************/
PT_EXPORT SmStatus my_test_sp_nurb(void)
{
//    SmContext sContext(pPool);
  SmContext sContext;

  {  // Test naturally bounded surfaces
    SmTArray<SmPoint3d> sPnts;
    my_load_surf1_pts(sPnts);
//    SmBSplineSurface *pNewBSS = my_create_nurb_surf(sContext,SmPoint3d(0,0,0),sPnts,6,4,3,3,1.0,0.5,0);
    SmBSplineSurface *pNewBSS = my_create_crease_surf(sContext,SmPoint3d(0,0,0),sbCreases,1.0,0.5,0.0);
    SmObjDelete sCU1(pNewBSS);

    // evaluate an array of UVPoints and test that GlobalPointSolve inverts the mapping for each eval
    SER(my_test_surface_point_intersect(*pNewBSS,pNewBSS->GetNaturalUVDomain(),33,33));

    // find nearest point to a sequence of test points
    ULONG lNumFound;
    SER(my_test_surface_point_extrema(*pNewBSS,pNewBSS->GetNaturalUVDomain(),SM_SO_MINIMIZE,10,10,2,lNumFound));
    SM_ASSERT(lNumFound == 200);

    // transform surface location
    SmAxis2Placement sA2P2;
    sA2P2.SetCanonical(SmPoint3d(0,7,-7),SmVector3d(1,0,0),SmVector3d(0,0,1));
    pNewBSS->Transform(sA2P2);

    // rerun the evaluate and globalPointSolve inverse mapping test
    SER(my_test_surface_point_intersect(*pNewBSS,pNewBSS->GetNaturalUVDomain(),33,33));

    // rerun the nearest point test
    SER(my_test_surface_point_extrema(*pNewBSS,pNewBSS->GetNaturalUVDomain(),SM_SO_NORMALIZE,10,2,10,lNumFound));
    // Temp: the new solvers return only the closest result for Normalize:
    SM_ASSERT((int)lNumFound == ( sbCreases ? 243 : 156 ));
    // Note, this number changes slightly on occasion.
    // See changes of 4/25/08; 10/5/09; 1/11/12; 3/26/13 to SmSurfacePointSolve.cpp.
  }

  {  // Test domain trimmed surfaces
    SmTArray<SmPoint3d> sPnts;
    my_load_surf1_pts(sPnts);
    SmBSplineSurface *pNewBSS = my_create_crease_surf(sContext,SmPoint3d(-8,0,0),sbCreases,1.0,0.5,0.0);
    SmObjDelete sCU1(pNewBSS);

    // reduce the target domain
    SmExtent2d sUVDom = pNewBSS->GetNaturalUVDomain();
    sUVDom.SetMinMax(sUVDom.Evaluate(0.1,0.1),sUVDom.Evaluate(0.9,0.9));

    // run evaluate/GlobalPointSolve inverse mapping on reduced domain
    SER(my_test_surface_point_intersect(*pNewBSS,sUVDom,33,33));

    // run closest point test on reduced domain
    ULONG lNumFound;
    SER(my_test_surface_point_extrema(*pNewBSS,sUVDom,SM_SO_MINIMIZE,10,10,2,lNumFound));
    SM_ASSERT(lNumFound == 200);

    // move the surface
    SmAxis2Placement sA2P2;
    sA2P2.SetCanonical(SmPoint3d(0,7,-7),SmVector3d(1,0,0),SmVector3d(0,0,1));
    pNewBSS->Transform(sA2P2);

    // reduce the target domain
    SmExtent2d sUVDom2 = pNewBSS->GetNaturalUVDomain();
    sUVDom2.SetMinMax(sUVDom2.Evaluate(0.1,0.1),sUVDom.Evaluate(0.9,0.9));

    // repeat evaluate/GlobalPointSolve test
    SER(my_test_surface_point_intersect(*pNewBSS,sUVDom2,33,33));

    // repeat closest point test
    SER(my_test_surface_point_extrema(*pNewBSS,sUVDom2,SM_SO_NORMALIZE,10,2,10,lNumFound));
    // Temp: the new solvers return only the closest result for Normalize:
    SM_ASSERT((int)lNumFound == ( sbCreases ? 171 : 86));
    // See changes of 4/25/08; 10/5/09; 1/11/12; 3/26/13 to SmSurfacePointSolve.cpp.
  }

  { // test thinly closed surface
      SmBSplineSurface* pTestSrf = NULL ;
      SER(my_surface_from_file(sContext, _T("../../TestFiles/pt_TestFiles/Surface.sms"),pTestSrf));
      SmObjDelete sCleanTSrf( pTestSrf );

      SmPoint3d pt1(-1.0616429349692744,-20.167126109553802,20.020823739437560 );
      SmPoint3d pt2(-1.1583434957685239, -19.953438722680168, 20.005921768970701 );
      SmPoint3d pt3(-4.5941574701649142,-9.6528635542995485,19.290949275641431 );
      SmPoint3d pt4( -0.83908714000092477,-20.645564191951244,20.054070584952139 );
      SmPoint3d pt5( -0.22169184902399575,-21.752883811327614,20.077084827504891 );
      SmPoint3d pt6( 0.097876793468328019,-21.633397717952313,18.978461556867217 );
      SmPoint3d pt7( -0.25257593806641893,-20.10880763638518,19.018298541213202 );
      SmPoint3d pt8( -0.32629818190650922,-19.760345509317247,19.026748453308592 );
      SmPoint3d pt9( -0.66892154856885222,-17.879224383359258,19.072647185802911 );
      SmPoint3d pt10( -0.83758573462226327,-20.648619822796935,20.054270506974774 );
      SmPoint3d pt11( -0.22211826515506564,-21.75240325273451,20.077401022086111 );
      SmPoint3d pt12( 0.097878601273081345,-21.633405764037899,18.978461275445223 );
      SmPoint3d pt13( -0.25280760311075551,-20.107797610675622,19.018327728721825 );
      SmPoint3d pt14( -0.32651463102618838,-19.7592999915767,19.026773809915298 );
      SmPoint3d pt15( -0.40219453853645376,-19.384417128948424,19.035877125510886 );

      SER(my_test_surface_closest_point(*pTestSrf,pt1));
      SER(my_test_surface_closest_point(*pTestSrf,pt2));
      SER(my_test_surface_closest_point(*pTestSrf,pt3));
      SER(my_test_surface_closest_point(*pTestSrf,pt4));
      SER(my_test_surface_closest_point(*pTestSrf,pt5));
      SER(my_test_surface_closest_point(*pTestSrf,pt6));
      SER(my_test_surface_closest_point(*pTestSrf,pt7));
      SER(my_test_surface_closest_point(*pTestSrf,pt8));
      SER(my_test_surface_closest_point(*pTestSrf,pt9));
      SER(my_test_surface_closest_point(*pTestSrf,pt10));
      SER(my_test_surface_closest_point(*pTestSrf,pt11));
      SER(my_test_surface_closest_point(*pTestSrf,pt12));
      SER(my_test_surface_closest_point(*pTestSrf,pt13));
      SER(my_test_surface_closest_point(*pTestSrf,pt14));
      SER(my_test_surface_closest_point(*pTestSrf,pt15));
  }

//    pPool->Free();
//    delete pPool; pPool = NULL ;

    return SM_SUCCESS;

} // end my_test_sp_nurb

/***********************************************************************
PURPOSE ---

USAGE NOTES ---
***********************************************************************/
PT_EXPORT SmStatus my_test_sp_analy(void)
{

//    SmContext sContext(pPool);
    SmContext sContext;

    SmBSplineSurface *pCylBSS = NULL ;
    SmAxis2Placement sA2P;

    if (TRUE) {
        SmTArray<SmPoint3d> sPnts1;
        sPnts1.Add(SmPoint3d(0,0.1,0));
        sPnts1.Add(SmPoint3d(0.3,0.5,0));
        sPnts1.Add(SmPoint3d(0.5,1,0.0));
        sPnts1.Add(SmPoint3d(0.6,0.3,0.0));
        sPnts1.Add(SmPoint3d(1.0,1.0,0.0));
        sPnts1.Add(SmPoint3d(1.3,0.3,0.0));
        sPnts1.Add(SmPoint3d(1.5,0.5,0));
        SmBSplineCurve *pNurb1 = my_create_nurb(sContext,SmPoint3d(0,0,0),sPnts1,3,1,0.5,0);
        SmObjDelete sCU3(pNurb1);
        SmBSplineSurface *pBSS = NULL ;
        SER(SmBSplineSurface::CreateSurfOfRevolution(sContext,pNurb1,SmPoint3d(0,2,0),SmVector3d(1,1,0),
            360.0,pBSS));
        SmObjDelete sCleanBSS( pBSS );
        SmSurface *pSurface;
        SER(pBSS->CopyAndAddAnalytics(sContext,pSurface)); NER(pSurface);
        SmSurfOfRevolution *pRev = (SmSurfOfRevolution*)pSurface;
        SmObjDelete sCleanSR( pRev );
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
            smgfx_SetColor(0, 0, 1);
            pRev->DrawUV(5, 5);
            smgfx_SetColor(1, 0, 0);
        }
#endif // SM_GFX_CODE
#ifdef SM_DEBUG_CODE
        clock_t start = clock();
#endif // SM_DEBUG_CODE
        SmSolution sSData[16];
        SmSolutionArray sSolutions(16,sSData);
        SmExtent2d sAnalDomain = pRev->GetSTEPUVDomain();
        SmExtent2d sUVDomain = pRev->GetNaturalUVDomain();
        ULONG lNum = 100;
        for (ULONG iu=0; iu<=lNum; iu++) {
            for (ULONG iv=0; iv<=lNum; iv++) {
                SmPoint2d sUV = sUVDomain.Evaluate(iu/(lNum*1.0),iv/(lNum*1.0));
                SmPoint2d sUVStep = sAnalDomain.Evaluate(iu/(lNum*1.0),iv/(lNum*1.0));
                SmPoint2d sUVStep2;
                SER(pRev->ConvertUVFromSTEPToNURBS(sUVStep,sUV));
                SER(pRev->ConvertUVFromNURBSToSTEP(sUV,sUVStep2));
                if (sUVStep.DistanceBetween(sUVStep2) > SM_EFF_ZERO) {
                    SE(SM_ERR);
                }
                SmPoint3d sPnt, sPnt2;
                SER(pRev->EvaluateSTEPPoint(sUVStep,sPnt));
                SER(pRev->EvaluatePoint(sUV,sPnt2));
                if (sPnt.DistanceBetween(sPnt2) > SM_EFF_ZERO) {
                    SE(SM_ERR);
                }
                    SER(pRev->GlobalPointSolve(sUVDomain,SM_SO_INTERSECT,sPnt,SM_ZONE_TOL_3D,
                        NULL,SM_SR_ALL,sSolutions));
//                    SER(pRev->DropPointFast(sUVDomain,SM_SO_INTERSECT,sPnt,NULL,SM_ZONE_TOL_3D,
//                        NULL,SM_SR_ALL,sSolutions));
                if (sSolutions.GetSize() == 0) {
                    SER(SM_ERR);
                }
                for (ULONG k=0; k<sSolutions.GetSize(); k++) {
                    SmSolution & rSol = sSolutions[k];
                    sUV.x = rSol.m_vStart[0];
                    sUV.y = rSol.m_vStart[1];
                    SER(pRev->EvaluatePoint(sUV,sPnt2));
                    if (sPnt.DistanceBetween(sPnt2) > SM_EFF_ZERO) {
                        SE(SM_ERR);
                    }
                }

#ifdef SM_GFX_CODE
//                sPnt.Draw();
#endif // SM_GFX_CODE
            }
        }
#ifdef SM_DEBUG_CODE
        clock_t finish = clock();
        sm_PrintTime(_T("1,000,000 SmSurfOfRevolution STEP Evaluations"),start,finish);
#endif // SM_DEBUG_CODE
        return SM_SUCCESS;

    }

    sA2P.SetCanonical(SmPoint3d(-5,0,0),SmVector3d(1,0,0),SmVector3d(0,1,0));
    SER(SmBSplineSurface::CreateConePatch(sContext,
        sA2P,1.0,1.0,0.0,360.0,1.0,SM_CO_QUADRATIC,pCylBSS));
    SmObjDelete sCU1(pCylBSS);
#ifdef SM_GFX_CODE
    if (smGet_DoGraphics()) {
        pCylBSS->DrawUV(2, 2);
    }
#endif // SM_GFX_CODE
    ULONG lNumFound;
    SER(my_test_surface_point_intersect(*pCylBSS,pCylBSS->GetNaturalUVDomain(),8,8));
    SER(my_test_surface_point_extrema(*pCylBSS,pCylBSS->GetNaturalUVDomain(),SM_SO_MINIMIZE,6,6,6,lNumFound));
    SM_ASSERT(lNumFound == 216);

    sA2P.SetCanonical(SmPoint3d(-5,5,0),SmVector3d(1,0,0),SmVector3d(0,1,0));
    SER(SmBSplineSurface::CreateConePatch(sContext,
        sA2P,1.0,1.0,0.0,180.0,1.0,SM_CO_QUADRATIC,pCylBSS));
    SmObjDelete sCU2(pCylBSS);
#ifdef SM_GFX_CODE
    if (smGet_DoGraphics()) {
        pCylBSS->DrawUV(2, 2);
    }
#endif // SM_GFX_CODE
    SER(my_test_surface_point_intersect(*pCylBSS,pCylBSS->GetNaturalUVDomain(),8,8));
    SER(my_test_surface_point_extrema(*pCylBSS,pCylBSS->GetNaturalUVDomain(),SM_SO_MINIMIZE,6,6,6,lNumFound));
    SM_ASSERT(lNumFound == 216);

    sA2P.SetCanonical(SmPoint3d(0,5,0),SmVector3d(1,0,0),SmVector3d(0,1,0));
    SER(SmBSplineSurface::CreateConePatch(sContext,
        sA2P,1.0,1.0,0.0,360.0,1.0,SM_CO_QUINTIC,pCylBSS));
    SmObjDelete sCU3(pCylBSS);
#ifdef SM_GFX_CODE
    if (smGet_DoGraphics()) {
        pCylBSS->DrawUV(2, 2);
    }
#endif // SM_GFX_CODE
    SER(my_test_surface_point_intersect(*pCylBSS,pCylBSS->GetNaturalUVDomain(),8,8));
    SER(my_test_surface_point_extrema(*pCylBSS,pCylBSS->GetNaturalUVDomain(),SM_SO_MINIMIZE,6,6,6,lNumFound));
    SM_ASSERT(lNumFound == 216);

    sA2P.SetCanonical(SmPoint3d(5,5,0),SmVector3d(1,0,0),SmVector3d(0,1,0));
    SER(SmBSplineSurface::CreateConePatch(sContext,
        sA2P,0.0,1.0,0.0,360.0,1.0,SM_CO_QUINTIC,pCylBSS));
    SmObjDelete sCU4(pCylBSS);
#ifdef SM_GFX_CODE
    if (smGet_DoGraphics()) {
        pCylBSS->DrawUV(2, 2);
    }
#endif // SM_GFX_CODE
    SER(my_test_surface_point_intersect(*pCylBSS,pCylBSS->GetNaturalUVDomain(),8,8));
    SER(my_test_surface_point_extrema(*pCylBSS,pCylBSS->GetNaturalUVDomain(),SM_SO_MINIMIZE,6,6,6,lNumFound));
    SM_ASSERT(lNumFound == 216);

    sA2P.SetCanonical(SmPoint3d(-5,-5,0),SmVector3d(1,0,0),SmVector3d(0,1,0));
    SER(SmBSplineSurface::CreateConePatch(sContext,
        sA2P,1.0,0.0,0.0,360.0,1.0,SM_CO_QUINTIC,pCylBSS));
    SmObjDelete sCU5(pCylBSS);
#ifdef SM_GFX_CODE
    if (smGet_DoGraphics()) {
        pCylBSS->DrawUV(2, 2);
    }
#endif // SM_GFX_CODE
    SER(my_test_surface_point_intersect(*pCylBSS,pCylBSS->GetNaturalUVDomain(),8,8));
    SER(my_test_surface_point_extrema(*pCylBSS,pCylBSS->GetNaturalUVDomain(),SM_SO_MINIMIZE,6,6,6,lNumFound));
    SM_ASSERT(lNumFound == 216);

//    pPool->Free();
//    delete pPool; pPool = NULL ;

    return SM_SUCCESS;

} // end my_test_sp_analy

/***********************************************************************
PURPOSE ---

USAGE NOTES ---
***********************************************************************/
PT_EXPORT SmStatus my_test_projection(void)
{
//    SmContext sContext(pPool);
    SmContext sContext;

    ULONG lCount;
    SmBSplineSurface *pNewBSS3 = NULL ;
    SER(my_surface_from_file(sContext, _T("../../TestFiles/pt_TestFiles/shovel.srf"),pNewBSS3));

#ifdef SM_GFX_CODE
    if ( smGet_DoGraphics() )
    {
        smgfx_SetColor( 0, 0, 0 );
        pNewBSS3->DrawUV( 1, 1 );
        smgfx_SetColor( 1, 0, 0 );
    }
#endif // SM_GFX_CODE
    SER(my_test_parallel_projection(*pNewBSS3,pNewBSS3->GetNaturalUVDomain(),20,lCount));
    SmObjDelete sCU3(pNewBSS3);


    SmTArray<SmPoint3d> sPnts;
    sPnts.Add(SmPoint3d(0,0,0));
    sPnts.Add(SmPoint3d(1,0.1,0.4));
    sPnts.Add(SmPoint3d(2,0.2,0.3));
    sPnts.Add(SmPoint3d(3,0.3,0));
    sPnts.Add(SmPoint3d(0,1,0.2));
    sPnts.Add(SmPoint3d(1,1,1.5));
    sPnts.Add(SmPoint3d(2,1,-1.6));
    sPnts.Add(SmPoint3d(3,1,0.2));
    sPnts.Add(SmPoint3d(0,2,0.3));
    sPnts.Add(SmPoint3d(1,2,-1.6));
    sPnts.Add(SmPoint3d(2,2,1.5));
    sPnts.Add(SmPoint3d(3,2,0.4));
    sPnts.Add(SmPoint3d(0,3,0.4));
    sPnts.Add(SmPoint3d(1,3,1.2));
    sPnts.Add(SmPoint3d(2,3,1.1));
    sPnts.Add(SmPoint3d(3,3,-0.7));
    sPnts.Add(SmPoint3d(0,4,0.4));
    sPnts.Add(SmPoint3d(1,4,0.2));
    sPnts.Add(SmPoint3d(2,4,-0.4));
    sPnts.Add(SmPoint3d(3,4,0.5));
    SmBSplineSurface *pNewBSS = my_create_nurb_surf(sContext,SmPoint3d(0,-5,0),sPnts,5,4,3,3,0.0,0.0,0);
    SmObjDelete sCU1(pNewBSS);

    SmTArray<SmPoint3d> sPnts2;
    sPnts2.Add(SmPoint3d(0,0,0.1));
    sPnts2.Add(SmPoint3d(1,0.1,0.3));
    sPnts2.Add(SmPoint3d(2,0.2,0.4));
    sPnts2.Add(SmPoint3d(3,0.3,0));
    sPnts2.Add(SmPoint3d(0,1,0.3));
    sPnts2.Add(SmPoint3d(1,1,0.6));
    sPnts2.Add(SmPoint3d(2,1,-1.5));
    sPnts2.Add(SmPoint3d(3,1,-1.2));
    sPnts2.Add(SmPoint3d(0,2,0.4));
    sPnts2.Add(SmPoint3d(1,2,0.5));
    sPnts2.Add(SmPoint3d(2,2,-1.6));
    sPnts2.Add(SmPoint3d(3,2,-1.3));
    sPnts2.Add(SmPoint3d(0,3,0.7));
    sPnts2.Add(SmPoint3d(1,3,-1.1));
    sPnts2.Add(SmPoint3d(2,3,-1.2));
    sPnts2.Add(SmPoint3d(3,3,0.4));
    sPnts2.Add(SmPoint3d(0,4,0.7));
    sPnts2.Add(SmPoint3d(1,4,0.1));
    sPnts2.Add(SmPoint3d(2,4,0.2));
    sPnts2.Add(SmPoint3d(3,4,0.6));
    SmBSplineSurface *pNewBSS2 = my_create_nurb_surf(sContext,SmPoint3d(5,5,0),sPnts2,5,4,3,3,0,0,0);
    SmObjDelete sCU2(pNewBSS2);

#ifdef SM_GFX_CODE

    smgfx_SetColor(0,0,0.8);
#endif // SM_GFX_CODE
    SER(my_test_parallel_projection(*pNewBSS,pNewBSS->GetNaturalUVDomain(),10,lCount));
    SM_ASSERT(lCount == 15);

#ifdef SM_GFX_CODE
    smgfx_SetColor(0,0,0.8);
#endif // SM_GFX_CODE
    SER(my_test_parallel_projection(*pNewBSS2,pNewBSS2->GetNaturalUVDomain(),20,lCount));
    SM_ASSERT(lCount == 29);

#ifdef SM_GFX_CODE
    if ( smGet_DoGraphics() )
    {
    }
#endif // SM_GFX_CODE

    return SM_SUCCESS;

} // end my_test_projection

/***********************************************************************
PURPOSE ---

USAGE NOTES ---
***********************************************************************/
PT_EXPORT SmStatus my_test_section(void)
{

    SmContext sContext;
    ULONG lCount;

    if (TRUE) {
        SmBSplineSurface *pNewBSS3 = NULL ;
        SER(my_surface_from_file(sContext, _T("../../TestFiles/pt_TestFiles/shovel.srf"),pNewBSS3));

#ifdef SM_GFX_CODE
        if ( smGet_DoGraphics() )
        {
            smgfx_SetColor( 0, 0, 0 );
            pNewBSS3->DrawUV( 1, 1 );
            smgfx_SetColor( 1, 0, 0 );
        }
#endif // SM_GFX_CODE
        SER(my_test_planar_section(*pNewBSS3,pNewBSS3->GetNaturalUVDomain(),SmVector3d(0,0,1),40,lCount));
        SmObjDelete sCU3(pNewBSS3);
    }

    if (TRUE) {
        SmTArray<SmPoint3d> sPnts;
        sPnts.Add(SmPoint3d(0,0,0));
        sPnts.Add(SmPoint3d(1,0.1,0.4));
        sPnts.Add(SmPoint3d(2,0.2,0.3));
        sPnts.Add(SmPoint3d(3,0.3,0));
        sPnts.Add(SmPoint3d(0,1,0.2));
        sPnts.Add(SmPoint3d(1,1,1.5));
        sPnts.Add(SmPoint3d(2,1,1.6));
        sPnts.Add(SmPoint3d(3,1,0.2));
        sPnts.Add(SmPoint3d(0,2,0.3));
        sPnts.Add(SmPoint3d(1,2,1.6));
        sPnts.Add(SmPoint3d(2,2,1.5));
        sPnts.Add(SmPoint3d(3,2,0.4));
        sPnts.Add(SmPoint3d(0,3,0.4));
        sPnts.Add(SmPoint3d(1,3,1.2));
        sPnts.Add(SmPoint3d(2,3,1.1));
        sPnts.Add(SmPoint3d(3,3,0.7));
        sPnts.Add(SmPoint3d(0,4,0.4));
        sPnts.Add(SmPoint3d(1,4,0.2));
        sPnts.Add(SmPoint3d(2,4,0.1));
        sPnts.Add(SmPoint3d(3,4,0.5));
        SmBSplineSurface *pNewBSS = my_create_nurb_surf(sContext,SmPoint3d(0,-5,0),sPnts,5,4,3,3,0.0,0.0,0);
        SmObjDelete sCU1(pNewBSS);

#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
            sm_GraphicsLoop();
        }
#endif // SM_GFX_CODE
        SER(my_test_planar_section(*pNewBSS,pNewBSS->GetNaturalUVDomain(),SmVector3d(0,0,1),10,lCount));
        SM_ASSERT(lCount == 10); // gwc: was 8 but intersector improved to give two degenerate (point) curve
#ifdef SM_GFX_CODE               //      results that used to be missing
        if (smGet_DoGraphics()) {
            sm_GraphicsLoop();
        }
#endif // SM_GFX_CODE               
        SER(my_test_planar_section(*pNewBSS,pNewBSS->GetNaturalUVDomain(),SmVector3d(1,1,0),10,lCount));
        SM_ASSERT(lCount == 10); // gwc: was 8 but intersector improved to give two degenerate (point) curve
    }                            //      results that used to be missing

    if (TRUE) {
        SmTArray<SmPoint3d> sPnts2;
        sPnts2.Add(SmPoint3d(0,0,0.1));
        sPnts2.Add(SmPoint3d(1,0.1,0.3));
        sPnts2.Add(SmPoint3d(2,0.2,0.4));
        sPnts2.Add(SmPoint3d(3,0.3,0));
        sPnts2.Add(SmPoint3d(0,1,0.3));
        sPnts2.Add(SmPoint3d(1,1,0.6));
        sPnts2.Add(SmPoint3d(2,1,-1.5));
        sPnts2.Add(SmPoint3d(3,1,-1.2));
        sPnts2.Add(SmPoint3d(0,2,0.4));
        sPnts2.Add(SmPoint3d(1,2,0.5));
        sPnts2.Add(SmPoint3d(2,2,-1.6));
        sPnts2.Add(SmPoint3d(3,2,-1.3));
        sPnts2.Add(SmPoint3d(0,3,0.7));
        sPnts2.Add(SmPoint3d(1,3,-1.1));
        sPnts2.Add(SmPoint3d(2,3,-1.2));
        sPnts2.Add(SmPoint3d(3,3,0.4));
        sPnts2.Add(SmPoint3d(0,4,0.7));
        sPnts2.Add(SmPoint3d(1,4,0.1));
        sPnts2.Add(SmPoint3d(2,4,0.2));
        sPnts2.Add(SmPoint3d(3,4,0.6));
        SmBSplineSurface *pNewBSS2 = my_create_nurb_surf(sContext,SmPoint3d(5,-5,0),sPnts2,5,4,3,3,0,0,0);
        SmObjDelete sCU2(pNewBSS2);

#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
            sm_GraphicsLoop();
        }
#endif // SM_GFX_CODE
        SER(my_test_planar_section(*pNewBSS2,pNewBSS2->GetNaturalUVDomain(),SmVector3d(0,0,1),20,lCount));
        SM_ASSERT(lCount == 21); // used to be 20: but cleaned up xsects now count 21 - verified visually
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
            sm_GraphicsLoop();
        }
#endif // SM_GFX_CODE
        SER(my_test_planar_section(*pNewBSS2,pNewBSS2->GetNaturalUVDomain(),SmVector3d(1,1,1),10,lCount));
        SM_ASSERT(lCount == 7);
    }

#ifdef SM_GFX_CODE
    if (smGet_DoGraphics()) {
    }
#endif // SM_GFX_CODE

    return SM_SUCCESS;

} // end my_test_section


// Static helper for my_test_surface_interface.
//
static SmBSplineCurve * create_curve( const SmContext & crContext, int which )
{
  SmBSplineCurve *pCrv = NULL;
  SmTArray< SmPoint3d > sPoints;
  if ( which == 1 ) {
      sPoints.Add( SmPoint3d(  1, 1, 1 ));
      sPoints.Add( SmPoint3d(  5, 1, 1 ));
      sPoints.Add( SmPoint3d(  5, 5, 1 ));
      sPoints.Add( SmPoint3d(  9, 5, 1 ));
  } else {
      sPoints.Add( SmPoint3d( 0, 0, 2 ));
      sPoints.Add( SmPoint3d( 4, 0, 2 ));
      sPoints.Add( SmPoint3d( 4, 4, 2 ));
      sPoints.Add( SmPoint3d( 8, 4, 2 ));
  }

  SmInterpolationType ePz = SM_IT_CHORDLENGTH;  // enum: in SmCurveTypes.h

  SmBSplineCurve::InterpolatePoints( crContext, sPoints, NULL, 3, NULL, NULL, FALSE, ePz, pCrv );

  return pCrv;
}
/***********************************************************************
PURPOSE --- Test SmSurface interface

USAGE NOTES ---
   This was written for the Java project: we need tests for all methods
   that are to be wrapped.  This is not testing the functionality of all
   the methods, only that they are being called correctly.  The list is:

  ApproximatePoints
  CreateSkinnedSurface
  CreateExtendedSurface1
  ApproximateOffsetSurface

  GetNaturalUVDomain
  GetKnotsPointers
  GetControlPointNet
  IsRational
  GetDegree
  IsPeriodic

  EvaluatePoint
  EvaluatePointOnExtendedSurface
  EvaluateNormal

  DropPoint
  GlobalLineIntersect
  LocalLineIntersect
  GlobalCurveIntersect
  GlobalSurfaceIntersect
  GlobalPointSolve
  GlobalCurveSolve
  GlobalSurfaceSolve

  Reparameterize
  SwapUV
  CreateIsoParametricCurve


enums:

in SmTypes.h:
 enum SmSurfParamType {
     SM_SP_U,
     SM_SP_V,
     SM_SP_UNKNOWN,
     SM_SP_BOTH,
     SM_SP_NEITHER,
     SM_SP_UMIN,
     SM_SP_VMIN,
     SM_SP_UMAX,
     SM_SP_VMAX
 };
***********************************************************************/
SmStatus my_test_surface_interface()
{
  SmContext sContext;

//##test ApproximatePoints, with default argument
  SmBSplineSurface *pNewSurf = NULL;
  SmTArray< SmPoint3d > sPts;
  my_load_surf1_pts( sPts );

  double dTol = 0.001;

  SmStatus eStat = SmBSplineSurface::ApproximatePoints( sContext,
                                      sPts,  // ordered: P[row][col] = p[row*lNumCols + col]
                                      6, 4, 3, 3, &dTol, pNewSurf, FALSE );

  if ( eStat != SM_SUCCESS )
    { SER( SM_ERR ); }
  if ( pNewSurf == NULL )
    { SER( SM_ERR ); }

#ifdef SM_DEBUG_CODE
  SmBoolean bDebugMe = FALSE;
  if (smGet_DoGraphics()) {
      if (bDebugMe) {
          pNewSurf->Dump();
          smgfx_SetLook(1, 2, 0, 0, 1); pNewSurf->DrawUV(4, 4); sm_GraphicsLoop();
          sm_GraphicsLoop();
      }
  }
#endif

//##test GetDegree
  ULONG lDeg = pNewSurf->GetDegree( SM_SP_U );
  if ( lDeg != 3 )
    { SER( SM_ERR ); }

//##test GetNaturalUVDomain
  SmExtent2d sDomain = pNewSurf->GetNaturalUVDomain();
  if ( sDomain.GetUMin() != 0.0 )
    { SER( SM_ERR ); }

//##test GetControlPointNet
  SmTArray< SmPoint3d > sCtrlPts;
  SmTArray< double > sWeights;
  ULONG lNumU = 0, lNumV = 0;

  eStat = pNewSurf->GetControlPointNet( lNumU, lNumV, sCtrlPts, sWeights );

  if ( eStat != SM_SUCCESS )
    { SER( SM_ERR ); }
  if ( lNumU != 6 )
    { SER( SM_ERR ); }
  if ( sCtrlPts.GetSize() != 24 )
    { SER( SM_ERR ); }
  if ( sWeights.GetSize() !=  0 )
    { SER( SM_ERR ); }

//##test GetKnotsPointers
  lNumU = lNumV = 0;
  double *pdKnotsU = NULL, *pdKnotsV = NULL;

  eStat = pNewSurf->GetKnotsPointers( lNumU, lNumV, pdKnotsU, pdKnotsV );

  if ( eStat != SM_SUCCESS )
    { SER( SM_ERR ); }
  if ( lNumU != 10 )
    { SER( SM_ERR ); }
  if ( pdKnotsU[0] != 0.0 )
    { SER( SM_ERR ); }

//##test IsRational
  SmBoolean bFlag = pNewSurf->IsRational();
  if ( bFlag == TRUE )
    { SER( SM_ERR ); }

//##test IsPeriodic
  sDomain = pNewSurf->GetNaturalUVDomain();
  bFlag = pNewSurf->IsPeriodic( sDomain, SM_SP_U );
  if ( bFlag == TRUE )
    { SER( SM_ERR ); }

//##test EvaluatePoint
  sDomain = pNewSurf->GetNaturalUVDomain();
  SmPoint2d sUV = sDomain.Evaluate( 0.5, 0.5 );
  SmPoint3d sPt;
  pNewSurf->EvaluatePoint( sUV, sPt );

  double dEps = 1.0e-8;
  if ( smos_Fabs( sPt.x - 1.4454876380771180 ) > dEps )
    { SER( SM_ERR ); }

//##test EvaluatePointOnExtendedSurface
  sUV.Set( 1.2, 1.2 );
  pNewSurf->EvaluatePointOnExtendedSurface( sUV, sPt );
  if ( smos_Fabs( sPt.x - 3.3196332743395720 ) > dEps )
    { SER( SM_ERR ); }

//##test EvaluateNormal
  SmVector3d sNormal;
  sUV.Set( 0.5, 0.5 );
  pNewSurf->EvaluateNormal( sUV, FALSE, FALSE, sNormal );

  dEps = 1.0e-8;
  if ( smos_Fabs( sNormal.x + 0.081949394674615525 ) > dEps )
    { SER( SM_ERR ); }

//##test DropPoint, with default argument
  sPt.Set( 4.0, 3.0, 1.0 );
  double dDist;
  SmBoolean bIsMulti;
  SmSolverOperationType eOpType = SM_SO_NORMALIZE;
  eStat = pNewSurf->DropPoint( sPt, sDomain, NULL, bFlag, sUV, dDist,bIsMulti, eOpType );

  if ( eStat != SM_SUCCESS )
    { SER( SM_ERR ); }
  if ( bFlag == TRUE )  // This should fail with SM_SO_NORMALIZE
    { SER( SM_ERR ); }

//##test DropPoint, with no default argument
  eStat = pNewSurf->DropPoint( sPt, sDomain, NULL, bFlag, sUV, dDist, bIsMulti );

  if ( eStat != SM_SUCCESS )
    { SER( SM_ERR ); }
  if ( bFlag != TRUE )  // This should succeed with default, which is SM_SO_MINIMIZE
    { SER( SM_ERR ); }
  if ( sUV.y != 1.0 )
    { SER( SM_ERR ); }

//##test GlobalLineIntersect
  SmSolutionArray sSolutions;
  sPt.Set( 2, 2, 0 );
  sNormal.Set( 0, 0, 1 );
  eStat = pNewSurf->GlobalLineIntersect( sDomain, sPt, sNormal, NULL, FALSE, dTol, sSolutions );
  if ( eStat != SM_SUCCESS )
    { SER( SM_ERR ); }
  if ( sSolutions.GetSize() != 1 )
    { SER( SM_ERR ); }

//##test LocalLineIntersect
  SmExtent1d sInterval( -100, 100 );
  SmPoint2d sGuessUV( 0.5, 0.5 );
  double dParam;
  SmPoint3d sIntPt, sIntNorm;
  eStat = pNewSurf->LocalLineIntersect( sDomain, sPt, sNormal, sInterval, dTol, dEps, sGuessUV,
    bFlag, sUV, dParam, dDist, sIntPt, sIntNorm );

  if ( eStat != SM_SUCCESS )
    { SER( SM_ERR ); }
  if ( bFlag != TRUE )
    { SER( SM_ERR ); }
  if ( dDist > dEps )
    { SER( SM_ERR ); }

//##test GlobalCurveIntersect
  SmCurve *pCrv = create_curve( sContext, 1 );
  if ( pCrv == NULL )
    { SER( SM_ERR ); }
  SmExtent1d sCrvDomain = pCrv->GetNaturalInterval();
  eStat = pNewSurf->GlobalCurveIntersect( sDomain, *pCrv, sCrvDomain, dTol, sSolutions );

  delete pCrv; pCrv = NULL;
  if ( eStat != SM_SUCCESS )
    { SER( SM_ERR ); }
  if ( sSolutions.GetSize() != 1 )
    { SER( SM_ERR ); }

//##test GlobalSurfaceIntersect
  // Create another surface.
  sPts.ReSet();
  my_load_surf4_pts( sPts );
  SmBSplineSurface *pSurf2 = NULL;
  eStat = SmBSplineSurface::ApproximatePoints( sContext,
                                      sPts, 6, 4, 3, 3, &dTol, pSurf2, FALSE );
  if ( eStat != SM_SUCCESS || pSurf2 == NULL )
    { SER( SM_ERR ); }
  SmExtent2d sDom2 = pSurf2->GetNaturalUVDomain();
  SmBoolean sFlags[2];
  sFlags[0] = sFlags[1] = FALSE;
  SmTArray< SmCurve* > s3dCurves;
  SmTArray< double   > sDeviations;

  eStat = pNewSurf->GlobalSurfaceIntersect(sContext, 
                                           sDomain, 
                                           *pSurf2, 
                                           sDom2,
                                           sFlags, 
                                           SM_CAST_APPROXTOL3D_PTR(&dTol), 
                                           NULL, 
                                           &s3dCurves, 
                                           NULL, NULL, NULL, 
                                           &sDeviations );

  if ( eStat != SM_SUCCESS )
    { SER( SM_ERR ); }
  if ( s3dCurves.GetSize() != 2 )
    { SER( SM_ERR ); }
  if ( sDeviations[0] > dTol )
    { SER( SM_ERR ); }

#ifdef SM_DEBUG_CODE
  if (smGet_DoGraphics()) {
      if (bDebugMe) {
          smgfx_SetLook(1, 2, 0, 0, 1); pNewSurf->DrawUV(4, 4); sm_GraphicsLoop();
          smgfx_SetLook(1, 2, 0, 1, 1); pSurf2->DrawUV(4, 4); sm_GraphicsLoop();
          smgfx_SetLook(2, 4, 1, 0, 0);
          ULONG ii;
          for (ii = 0; ii < s3dCurves.GetSize(); ii++)
          {
              s3dCurves[ii]->Draw(); sm_GraphicsLoop();
          }
          sm_GraphicsLoop();
      }
  }
#endif

  delete pSurf2; pSurf2 = NULL;
  ULONG iii;
  for ( iii=0; iii<s3dCurves.GetSize(); iii++ )
    { delete s3dCurves[iii]; }
  s3dCurves.ReSet();

//##test GlobalPointSolve
  sIntPt.Set( 2, 2, 2 );
  SmSolverOperationType eSolverOp = SM_SO_MINIMIZE;
  SmSolutionRequestedType eSolType = SM_SR_ALL;

  eStat = pNewSurf->GlobalPointSolve( sDomain, eSolverOp, sIntPt,
                          dTol, NULL, eSolType, sSolutions );

  if ( eStat != SM_SUCCESS )
    { SER( SM_ERR ); }
  if ( sSolutions.GetSize() != 1 )
    { SER( SM_ERR ); }

//##test GlobalCurveSolve
  SmCurve *pCrv2 = create_curve( sContext, 2 );
  if ( pCrv2 == NULL )
    { SER( SM_ERR ); }
  sCrvDomain = pCrv2->GetNaturalInterval();
  eStat = pNewSurf->GlobalCurveSolve( sDomain, *pCrv2, sCrvDomain, eSolverOp,
                      dTol, NULL, NULL, eSolType, sSolutions );

  if ( eStat != SM_SUCCESS )
    { SER( SM_ERR ); }
  if ( sSolutions.GetSize() != 1 )
    { SER( SM_ERR ); }

  delete pCrv2; pCrv2 = NULL;

//##test GlobalSurfaceSolve
  sPts.ReSet();
  my_load_surf2_pts( sPts );
  pSurf2 = NULL;
  eStat = SmBSplineSurface::ApproximatePoints( sContext,
                                      sPts, 4, 4, 3, 3, &dTol, pSurf2, FALSE );
  if ( eStat != SM_SUCCESS || pSurf2 == NULL )
    { SER( SM_ERR ); }
  sDom2 = pSurf2->GetNaturalUVDomain();
  eSolverOp = SM_SO_MINIMIZE;
  eSolType = SM_SR_ALL;

  eStat = pNewSurf->GlobalSurfaceSolve( sDomain, *pSurf2, sDom2, eSolverOp,
              dTol, NULL, NULL, eSolType, sSolutions );

  if ( eStat != SM_SUCCESS )
    { SER( SM_ERR ); }
  if ( sSolutions.GetSize() != 1 )
    { SER( SM_ERR ); }

  delete pSurf2; pSurf2 = NULL;

//##test Reparameterize
  SmExtent2d sNewDomain( 1, 2, 3, 4 );

  eStat = pNewSurf->Reparameterize( sNewDomain );

  if ( eStat != SM_SUCCESS )
    { SER( SM_ERR ); }
  lNumU = lNumV = 0;
  pdKnotsU = NULL, pdKnotsV = NULL;
  eStat = pNewSurf->GetKnotsPointers( lNumU, lNumV, pdKnotsU, pdKnotsV );
  if ( eStat != SM_SUCCESS )
    { SER( SM_ERR ); }
  if ( pdKnotsU[0] != 1.0 )
    { SER( SM_ERR ); }

//##test SwapUV
  eStat = pNewSurf->SwapUV();
  lNumU = lNumV = 0;
  pdKnotsU = NULL, pdKnotsV = NULL;
  eStat = pNewSurf->GetKnotsPointers( lNumU, lNumV, pdKnotsU, pdKnotsV );
  if ( eStat != SM_SUCCESS )
    { SER( SM_ERR ); }
  if ( pdKnotsU[0] != 2.0 )
    { SER( SM_ERR ); }

//##test CreateIsoParametricCurve, with default argument
  SmSurfParamType eSurfParam = SM_SP_U;
  sNewDomain.SetMinMax( 1.5, 2.5, 2.5, 3.5 );
  SmBSplineCurve *pNewCurve = NULL;

  eStat = pNewSurf->CreateIsoParametricCurve( sContext, eSurfParam, 2.0, dTol, pNewCurve, &sNewDomain );

  if ( eStat != SM_SUCCESS )
    { SER( SM_ERR ); }
  if ( pNewCurve == NULL )
    { SER( SM_ERR ); }

  delete pNewCurve; pNewCurve = NULL;

//##test CreateIsoParametricCurve, with no default argument
  eSurfParam = SM_SP_U;

  eStat = pNewSurf->CreateIsoParametricCurve( sContext, eSurfParam, 2.0, dTol, pNewCurve );

  if ( eStat != SM_SUCCESS )
    { SER( SM_ERR ); }
  if ( pNewCurve == NULL )
    { SER( SM_ERR ); }

  delete pNewCurve; pNewCurve = NULL;

//##test CreateSkinnedSurface, with default argument
  // Create three isoparameteric curves from pNewSurf, and skin them.
  SmBSplineCurve *pCurve1 = NULL, *pCurve2 = NULL, *pCurve3 = NULL;
  // Make sure the parameterization is what we want.
  sNewDomain.SetMinMax( 0, 0, 1, 1 );
  eStat = pNewSurf->Reparameterize( sNewDomain );
  if ( eStat != SM_SUCCESS )
    { SER( SM_ERR ); }

  eSurfParam = SM_SP_U;
  dParam = 0.0;
  eStat = pNewSurf->CreateIsoParametricCurve( sContext, eSurfParam, dParam, dTol, pCurve1 );
  if ( eStat != SM_SUCCESS )
    { SER( SM_ERR ); }
  if ( pCurve1 == NULL )
    { SER( SM_ERR ); }
  dParam = 0.5;
  eStat = pNewSurf->CreateIsoParametricCurve( sContext, eSurfParam, dParam, dTol, pCurve2 );
  if ( eStat != SM_SUCCESS )
    { SER( SM_ERR ); }
  if ( pCurve2 == NULL )
    { SER( SM_ERR ); }
  dParam = 1.0;
  eStat = pNewSurf->CreateIsoParametricCurve( sContext, eSurfParam, dParam, dTol, pCurve3 );
  if ( eStat != SM_SUCCESS )
    { SER( SM_ERR ); }
  if ( pCurve3 == NULL )
    { SER( SM_ERR ); }

  SmTArray< SmBSplineCurve* > sCurves;
  sCurves.Add( pCurve1 );
  sCurves.Add( pCurve2 );
  sCurves.Add( pCurve3 );

  eSurfParam = SM_SP_U;
  SmBSplineSurface * pDerivSurfs[2];
  pDerivSurfs[0] = pDerivSurfs[1] = NULL;
  SmBSplineSurface *pSkinSurf = NULL;

  // Note: static method
  eStat = SmBSplineSurface::CreateSkinnedSurface( sContext, sCurves, TRUE, eSurfParam, dTol,
              NULL, NULL, FALSE, NULL, pDerivSurfs, pSkinSurf, 3 );

  if ( eStat != SM_SUCCESS )
    { SER( SM_ERR ); }
  if ( pSkinSurf == NULL )
    { SER( SM_ERR ); }

#ifdef SM_DEBUG_CODE
  if (smGet_DoGraphics()) {
      if (bDebugMe) {
          smgfx_Erase();
          smgfx_SetLook(2, 4, 1, 0, 0);
          ULONG kk;
          for (kk = 0; kk < sCurves.GetSize(); kk++)
          {
              sCurves[kk]->Draw(); sm_GraphicsLoop();
          }
          smgfx_SetLook(1, 2, 0, 0, 1); pSkinSurf->DrawUV(4, 4); sm_GraphicsLoop();
          sm_GraphicsLoop();
      }
  }
#endif

  delete pSkinSurf; pSkinSurf = NULL;


//##test CreateSkinnedSurface, with no default argument

  // Note: static method
  eStat = SmBSplineSurface::CreateSkinnedSurface( sContext, sCurves, TRUE, eSurfParam, dTol,
              NULL, NULL, FALSE, NULL, pDerivSurfs, pSkinSurf );

  if ( eStat != SM_SUCCESS )
    { SER( SM_ERR ); }
  if ( pSkinSurf == NULL )
    { SER( SM_ERR ); }

  delete pSkinSurf; pSkinSurf = NULL;
  ULONG jj;
  for ( jj=0; jj<sCurves.GetSize(); jj++ )
    { delete sCurves[jj]; }

//##test CreateExtendedSurface
  SmSurface *pExtSurf = NULL;
  SmContinuityType eCont = SM_CT_CINFINITY;

  pNewSurf->CreateExtendedSurface( sContext, 0.5, eCont, pExtSurf );

  if ( eStat != SM_SUCCESS )
    { SER( SM_ERR ); }
  if ( pExtSurf == NULL )
    { SER( SM_ERR ); }

#ifdef SM_DEBUG_CODE
    if(smGet_DoGraphics()) {
        if (bDebugMe) {
            smgfx_Erase();
            smgfx_SetLook(1, 2, 0, 0, 1); pNewSurf->DrawUV(4, 4); sm_GraphicsLoop();
            smgfx_SetLook(2, 4, 1, 0, 0); pExtSurf->DrawUV(4, 4); sm_GraphicsLoop();
            sm_GraphicsLoop();
        }
    }
#endif

  delete pExtSurf; pExtSurf = NULL;

//##test ApproximateOffsetSurface, with default argument
  SmBSplineSurface *pOffSurf = NULL;

  eStat = pNewSurf->ApproximateOffsetSurface( sContext, 0.2, dTol, pOffSurf, 1 );

  if ( eStat != SM_SUCCESS )
    { SER( SM_ERR ); }
  if ( pOffSurf == NULL )
    { SER( SM_ERR ); }

#ifdef SM_DEBUG_CODE
  if (smGet_DoGraphics()) {
      if (bDebugMe) {
          smgfx_Erase();
          smgfx_SetLook(1, 2, 0, 0, 1); pNewSurf->DrawUV(4, 4); sm_GraphicsLoop();
          smgfx_SetLook(2, 4, 1, 0, 0); pOffSurf->DrawUV(4, 4); sm_GraphicsLoop();
          sm_GraphicsLoop();
      }
  }
#endif

  delete pOffSurf; pOffSurf = NULL;

//##test ApproximateOffsetSurface, with no default argument
  pOffSurf = NULL;

  eStat = pNewSurf->ApproximateOffsetSurface( sContext, 0.2, dTol, pOffSurf );

  if ( eStat != SM_SUCCESS )
    { SER( SM_ERR ); }
  if ( pOffSurf == NULL )
    { SER( SM_ERR ); }

  delete pOffSurf; pOffSurf = NULL;


  // All done.
  delete pNewSurf; pNewSurf = NULL;

  return SM_SUCCESS;

} // end my_test_surface_interface

/***********************************************************************
PURPOSE ---

USAGE NOTES ---
***********************************************************************/
PT_EXPORT SmStatus my_test_bsplinesurface(void)
{
  my_test_surface_interface();

//    SmContext sContext(pPool);
    SmContext sContext;

    if (FALSE) {
        my_test_ss_nurb();
        return SM_SUCCESS;
    }

    if (FALSE) {
        my_test_drop_curve();
        return SM_SUCCESS;
    }

    if (FALSE) {
        my_test_cs_nurb();
        return SM_SUCCESS;
    }

    if (FALSE) {
        my_test_surface_suite();
        return SM_SUCCESS;
    }

    if (FALSE) {
        SER(my_test_csi_coincidence());
        return SM_SUCCESS;
    }
    if (FALSE) {
        SER(my_test_csi_analy());
        return SM_SUCCESS;
    }

    if (FALSE) {
        SER(my_test_csi_tangency());
    }


    SmBSplineSurface *pCylBSS = NULL ;
    SmAxis2Placement sA2P;
    sA2P.SetCanonical(SmPoint3d(-2,0,0),SmVector3d(1,0,0),SmVector3d(0,1,0));
    SER(SmBSplineSurface::CreateConePatch(sContext,
        sA2P,1.0,1.0,0.0,360.0,1.0,SM_CO_QUADRATIC,pCylBSS));
    SmObjDelete sCU1(pCylBSS);

//    pCylBSS->Draw();

    if (FALSE) {
        SER(my_test_surface_tessellation(*pCylBSS,pCylBSS->GetNaturalUVDomain(),0.01,20.0*SM_PI/180.0));
    }

    SmTArray<SmPoint3d> sPnts;
    sPnts.Add(SmPoint3d(0,0,0));
    sPnts.Add(SmPoint3d(1,0.1,0.4));
    sPnts.Add(SmPoint3d(2,0.2,0.3));
    sPnts.Add(SmPoint3d(3,0.3,0));
    sPnts.Add(SmPoint3d(0,1,0.2));
    sPnts.Add(SmPoint3d(1,1,0.5));
    sPnts.Add(SmPoint3d(2,1,0.6));
    sPnts.Add(SmPoint3d(3,1,0.2));
    sPnts.Add(SmPoint3d(0,2,0.3));
    sPnts.Add(SmPoint3d(1,2,0.6));
    sPnts.Add(SmPoint3d(2,2,0.5));
    sPnts.Add(SmPoint3d(3,2,0.4));
    sPnts.Add(SmPoint3d(0,3,0.4));
    sPnts.Add(SmPoint3d(1,3,0.2));
    sPnts.Add(SmPoint3d(2,3,0.1));
    sPnts.Add(SmPoint3d(3,3,0.7));
    sPnts.Add(SmPoint3d(0,4,0.4));
    sPnts.Add(SmPoint3d(1,4,0.2));
    sPnts.Add(SmPoint3d(2,4,0.1));
    sPnts.Add(SmPoint3d(3,4,0.7));
    SmBSplineSurface *pNewBSS = my_create_nurb_surf(sContext,SmPoint3d(0,0,0),sPnts,5,4,3,3,0,0,0);
    SmObjDelete sCU2(pNewBSS);

    if (TRUE) {
        double dArea;
        SER(pCylBSS->Area(pCylBSS->GetNaturalUVDomain(),1.0e-8,dArea));
        TCHAR sBuff[SM_TBLOCK_SIZE];
        SM_SPRINTF(sBuff,_T("Area = %16.16lf\n"),dArea);
        smos_WriteBuffer(sBuff);
        // return SM_SUCCESS; cbi why was this here?
    }

    if (FALSE) {
        SER(my_test_surface_tessellation(*pNewBSS,pNewBSS->GetNaturalUVDomain(),0.05,20.0*SM_PI/180.0));
    }

    if (FALSE) {
        ULONG lNumFound;
        SER(my_test_surface_curve_intersect(*pNewBSS,pNewBSS->GetNaturalUVDomain(),5,5,lNumFound));
    }

    if (TRUE) {
        SER(my_test_surface_point_intersect(*pNewBSS,pNewBSS->GetNaturalUVDomain(),33,33));
    }

    if (FALSE) { // Basic point dropping test
        ULONG lNumFound;
        SER(my_test_surface_point_extrema(*pNewBSS,pNewBSS->GetNaturalUVDomain(),SM_SO_MINIMIZE,10,10,10,lNumFound));
    }

    SmAxis2Placement sA2P2;
    sA2P2.SetCanonical(SmPoint3d(0,5,-5),SmVector3d(1,0,0),SmVector3d(0,0,1));
    pNewBSS->Transform(sA2P2);
#ifdef SM_GFX_CODE
    if ( smGet_DoGraphics() )
    {
        pNewBSS->DrawUV( 3, 3 );
    }
#endif // SM_GFX_CODE
    if (FALSE) { // Basic point dropping test
        ULONG lNumFound;
        SER(my_test_surface_point_extrema(*pNewBSS,pNewBSS->GetNaturalUVDomain(),SM_SO_MINIMIZE,10,10,10,lNumFound));
    }


    if (FALSE) { // Test geometric evaluation
        double G, H, k1, k2;
        SmVector3d sCV1, sCV2;
        SmPoint3d sEFG, sLMN;
        SER(pNewBSS->EvaluateGeometric(SmPoint2d(0.5,0.5),TRUE,TRUE,G,H,k1,k2,
            sEFG,sLMN,sCV1,sCV2));
        SER(pCylBSS->EvaluateGeometric(SmPoint2d(0.5,0.5),TRUE,TRUE,G,H,k1,k2,
            sEFG,sLMN,sCV1,sCV2));
    }

    if (FALSE) {
        double dArea;
        SER(pCylBSS->Area(pCylBSS->GetNaturalUVDomain(),1.0e-8,dArea));
        TCHAR sBuff[SM_TBLOCK_SIZE];
        SM_SPRINTF(sBuff,_T("Area = %16.16lf\n"),dArea);
        smos_WriteBuffer(sBuff);
    }

    if (FALSE) { // Basic surface derivative evaluation test
#ifdef SM_GFX_CODE
        smgfx_SetColor(0.7,0.6,0.3);
        //    pNewBSS->DrawNet();
        pNewBSS->Dump(TRUE);

        SmVector3d sPVV[2][2];
        SmPoint2d sUV(0.5,0.5);
        pNewBSS->Evaluate(sUV,1,1,TRUE,TRUE,TRUE,sPVV[0]);

        SmVector3d sPVVVV[3][3];
#ifdef SM_DEBUG_CODE
        clock_t start = clock();
#endif
        for (ULONG i=0; i<1000; i++) {
            pNewBSS->Evaluate(sUV,2,2,TRUE,TRUE,TRUE,sPVVVV[0]);
        }
#ifdef SM_DEBUG_CODE
        clock_t finish = clock();
        sm_PrintTime(_T("1000 Surface Derivative Evaluations"),start,finish);
#endif

        SmVector3d sPV13[1][3];
        pNewBSS->Evaluate(sUV,0,2,TRUE,TRUE,TRUE,sPV13[0]);

        SmVector3d sPV23[2][3];
        pNewBSS->Evaluate(sUV,1,2,TRUE,TRUE,TRUE,sPV23[0]);

        SmVector3d sPV31[3][1];
        pNewBSS->Evaluate(sUV,2,0,TRUE,TRUE,TRUE,sPV31[0]);

        SmVector3d sPV32[3][2];
        pNewBSS->Evaluate(sUV,2,1,TRUE,TRUE,TRUE,sPV32[0]);
#endif // SM_GFX_CODE
    }

    if (FALSE) { // basic surface bounding box test
        SmExtent3d sBBox2;
        SmPseudoBox sPBox2;
        SER(pNewBSS->CalculateBoundingBox(pNewBSS->GetNaturalUVDomain(),&sBBox2,&sPBox2));
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
            smgfx_SetColor(0,1,0);
            sBBox2.Draw();
            smgfx_SetColor(0,0,1);
            sPBox2.Draw();
            smgfx_SetColor(0,0,0);
        }
#endif // SM_GFX_CODE
    }

//    pPool->Free();
//    delete pPool; pPool = NULL ;

    return SM_SUCCESS;

} // end my_test_bsplinesurface

/***********************************************************************
PURPOSE ---

USAGE NOTES ---
***********************************************************************/
SmStatus my_test_analytic_surface
  (SmBSplineSurface * pSurface,        // in : tgt surface
   SmExtent2d       & rAnalDomain)     // in : trim surface to given AnalDomain
{
  SmContext sContext;
  const SmContext *pContext =   pSurface->GetContext()
                              ? pSurface->GetContext()
                              : &sContext ;
  {
    // make and delete a copy of the Surface
    SmSurface *pCopy;
    SER(pSurface->Copy(*pContext,pCopy));
    SmObjDelete sCleanCopy(pCopy);

    smos_WriteBuffer(_T("\n***************** Original ************************\n"));
    pSurface->Dump(TRUE);
    smos_WriteBuffer(_T("\n***************** Copy ************************\n"));
    pCopy->Dump(TRUE);
  }

  //
  SER(pSurface->AdjustSTEPUVDomain(rAnalDomain));
#ifdef SM_GFX_CODE
  if (smGet_DoGraphics()) {
      smgfx_Erase() ;
      smgfx_SetColor(0,0,1); pSurface->DrawUV(2,2); sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_GFX_CODE

  SmExtent2d sNurbDomain = pSurface->GetNaturalUVDomain();
  SmPoint2d  sUVMin      = sNurbDomain.GetMin();
  SmPoint2d  sUVMax      = sNurbDomain.GetMax();
  SmPoint2d  sSize       = sNurbDomain.GetSize();
  double     dStepU      = sSize.x/10.0;
  double     dStepV      = sSize.y/10.0;

  // for a grid of surface sample points -
  //  check UVPoint->[EvaluatePoint]->3DPoint->[GlobalPointSolveSTEP]
  for (ULONG ii=0; ii<=10; ii++)
    {
      double dU = ii*dStepU + sUVMin.x;
      if (ii == 10) dU = sUVMax.x;
      for (ULONG jj=0; jj<=10; jj++)
        {
          double dV = jj*dStepV + sUVMin.y;
          if (jj == 10) dV = sUVMax.y;
          SmPoint3d sPnt;

          // map UVPoint -> 3DPoint
          SER(pSurface->EvaluatePoint(SmPoint2d(dU,dV),sPnt));
          SmSolutionArray sSolutions;
          double dTolerance = SM_EFF_ZERO_SQRT;

          // map 3DPoint -> UVPoint
          SER(pSurface->GlobalPointSolveSTEP(rAnalDomain,SM_SO_INTERSECT,
                                             sPnt,dTolerance,NULL,SM_SR_ALL,sSolutions));

          // for every solution verify mapped value == original value
          for (ULONG j=0; j<sSolutions.GetSize(); j++)
            {
              SmPoint2d sAnalUV = SmPoint2d(sSolutions[j].m_vStart[0],
                                            sSolutions[j].m_vStart[1]);
              SmPoint3d sAnalPnt;
              SER(pSurface->EvaluateSTEPPoint(sAnalUV,sAnalPnt));
              double dDist = sAnalPnt.DistanceBetween(sPnt);
              if (dDist > dTolerance)
                {
                  SER(SM_ERR);
                }
            }
        } // end iter jj on a grid of surface sample points
    } // end iter ii on a grid of surface sample points

  // Test lifting a curve

  // Build a temporary step surface of the input surface
  SmSTEPSurface *pSTEPSrf = new(*pContext) SmSTEPSurface(*pSurface,FALSE);
  SmObjDelete    sClean(pSTEPSrf);

  // make a linear UV BSpline Curve running from Surface MinCorner to MaxCorner
  SmExtent2d      sSTEPUV = pSTEPSrf->GetNaturalUVDomain();
  SmBSplineCurve *pUVLine = NULL ;
  SmPoint3d sLLCorner(sSTEPUV.GetMin());
  SmPoint3d sURCorner(sSTEPUV.GetMax());
  SER(SmBSplineCurve::CreateLineSegment(*pContext,2,
                                        sLLCorner,
                                        sURCorner,
                                        pUVLine));
  SmObjDelete sClean2(pUVLine);
  SmExtent1d sLineIvl = pUVLine->GetNaturalInterval();

  // Lift the diagonal UVTrimCurve to a 3d Curve
  SmBSplineCurve *p3DCurve = NULL;
  double dMaxDistToSurf;
  SER(pSTEPSrf->LiftCurve(*pContext,sSTEPUV,*pUVLine,sLineIvl,0.00001,
                          dMaxDistToSurf,p3DCurve));

  if (p3DCurve == NULL) SER(SM_ERR);
  SmObjDelete sClean3(p3DCurve);

#ifdef SM_GFX_CODE
  if (smGet_DoGraphics()) { // dump and draw the curve
      p3DCurve->Dump(TRUE);
      smgfx_SetColor(1,0,0); p3DCurve->Draw(); sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;

    }
#endif // SM_GFX_CODE

  // all done
  return SM_SUCCESS;

} // end my_test_analytic_surface


/***********************************************************************
PURPOSE ---

USAGE NOTES ---
***********************************************************************/
PT_EXPORT SmStatus my_test_analytic_surfaces(void)
{
  SmContext sContext;
  SmBSplineSurface * pSurface = NULL;

  if (TRUE)
    { //Plane Tests
      SmPoint3d sOrigin(0.0,0.0,0.0);
      SmVector3d sX(0.0,1.0,0.0);
      SmVector3d sY(0.0,0.0,1.0);
      SmAxis2Placement sPos;
      sPos.SetCanonical(sOrigin,sX,sY);
      SmPlane * pNewPlane = NULL;
      SER(SmPlane::CreateCanonical(sContext,sPos,pNewPlane));
      pSurface = pNewPlane;
      SmObjDelete sSU(pSurface);
      SmPoint2d sUVMin(-1.0,-1.0);
      SmPoint2d sUVMax(1.0,1.0);
      SmExtent2d sAnalDomain(sUVMin,sUVMax);
      SER(my_test_analytic_surface(pSurface,sAnalDomain));
    }

  if (TRUE)
    { //Cylinder Tests
      SmPoint3d sOrigin(-1.0,0.0,0.0);
      SmVector3d sX(1.0,0.0,0.0);
      SmVector3d sY(0.0,1.0,0.0);
      SmAxis2Placement sPos;
      sPos.SetCanonical(sOrigin,sX,sY);
      SmCylinder * pNewCylinder = NULL;
      SER(SmCylinder::CreateCanonical(sContext,sPos,1.0,pNewCylinder));
      pSurface = pNewCylinder;
      SmObjDelete sSU(pSurface);
      SmPoint2d sUVMin = SmPoint2d(0.0,-1.0);
      SmPoint2d sUVMax = SmPoint2d(360.0,1.0);
      SmExtent2d sAnalDomain(sUVMin,sUVMax);
      SER(my_test_analytic_surface(pSurface,sAnalDomain));
    }

  if (TRUE)
    { //Cone Tests
      SmPoint3d sOrigin(0.0,1.0,0.0);
      SmVector3d sX(1.0,0.0,0.0);
      SmVector3d sY(0.0,-1.0,0.0);
      SmAxis2Placement sPos;
      sPos.SetCanonical(sOrigin,sX,sY);
      SmCone * pNewCone = NULL;
      SER(SmCone::CreateCanonical(sContext,sPos,1.0,45.0,pNewCone));
      pSurface = pNewCone;
      SmObjDelete sSU(pSurface);
      SmPoint2d sUVMin = SmPoint2d(0.0,-1.0);
      SmPoint2d sUVMax = SmPoint2d(360.0,1.0);
      SmExtent2d sAnalDomain(sUVMin,sUVMax);
      SER(my_test_analytic_surface(pSurface,sAnalDomain));
    }

  if (TRUE)
    { //0-rad base, semi-infinite Cone Tests
      SmPoint3d sOrigin(0.0,0.0,0.0);
      SmVector3d sX(1.0,0.0,0.0);
      SmVector3d sY(0.0,1.0,0.0);
      SmAxis2Placement sPos;
      sPos.SetCanonical(sOrigin,sX,sY);
      SmCone * pNewCone = NULL;
      SER(SmCone::CreateCanonical(sContext,sPos,0.0,45.0,pNewCone));
      pSurface = pNewCone;
      SmObjDelete sSU(pSurface);
      SmExtent2d sSTEPDomain = pSurface->GetSTEPUVDomain();
      SER(my_test_analytic_surface(pSurface,sSTEPDomain));
    }

  if (TRUE)
    { //0-rad base, inside-out, semi-infinite Cone Tests
      SmPoint3d sOrigin(0.0,0.0,0.0);
      SmVector3d sX(1.0,0.0,0.0);
      SmVector3d sY(0.0,1.0,0.0);
      SmAxis2Placement sPos;
      sPos.SetCanonical(sOrigin,sX,sY);
      SmCone * pNewCone = NULL;
      SER(SmCone::CreateCanonical(sContext,sPos,0.0,45.0,pNewCone, NULL, FALSE, TRUE));
      pSurface = pNewCone;
      SmObjDelete sSU(pSurface);
      SmExtent2d sSTEPDomain = pSurface->GetSTEPUVDomain();
      SER(my_test_analytic_surface(pSurface,sSTEPDomain));
    }

  if (TRUE)
    { //Torus Tests
      SmPoint3d sOrigin(2.0,2.0,0.0);
      SmVector3d sX(1.0,0.0,0.0);
      SmVector3d sY(0.0,1.0,0.0);
      SmAxis2Placement sPos;
      sPos.SetCanonical(sOrigin,sX,sY);
      SmTorus * pNewTorus = NULL;
      SER(SmTorus::CreateCanonical(sContext,sPos,1.0,0.25,pNewTorus));
      pSurface = pNewTorus;
      SmObjDelete sSU(pSurface);
      SmPoint2d sUVMin = SmPoint2d(0.0,0.0);
      SmPoint2d sUVMax = SmPoint2d(360.0,360.0);
      SmExtent2d sAnalDomain(sUVMin,sUVMax);
      SER(my_test_analytic_surface(pSurface,sAnalDomain));
    }

  if (TRUE)
    { //Sphere Tests
      SmPoint3d sOrigin(-2.0,2.0,0.0);
      SmVector3d sX(1.0,0.0,0.0);
      SmVector3d sY(0.0,1.0,0.0);
      SmAxis2Placement sPos;
      sPos.SetCanonical(sOrigin,sX,sY);
      SmSphere * pNewSphere = NULL;
      SER(SmSphere::CreateCanonical(sContext,sPos,1.0,pNewSphere));
      pSurface = pNewSphere;
      SmObjDelete sSU(pSurface);
      SmPoint2d sUVMin = SmPoint2d(0.0,-90.0);
      SmPoint2d sUVMax = SmPoint2d(90.0,90.0);
      SmExtent2d sAnalDomain(sUVMin,sUVMax);
      SER(my_test_analytic_surface(pSurface,sAnalDomain));
    }

  if (TRUE)
    { //Surface-of-Revolution Tests
      SmPoint3d sOrigin(1.0,0.0,0.0);
      SmVector3d sX(1.0,0.0,0.0);
      SmVector3d sY(0.0,0.0,1.0);
      SmAxis2Placement sPos;
      sPos.SetCanonical(sOrigin,sX,sY);
      SmParabola * pParabola = NULL;
      SmParabola::CreateCanonical(sContext,sPos,0.5,pParabola);
      SmExtent1d sCrvIvl(0.0,2.0);
      SmBSplineCurve * pCurve = pParabola;
      SER(pCurve->AdjustSTEPInterval(sCrvIvl));

      SmPoint3d sAxisPoint(0.0,0.0,0.0);
      SmVector3d sAxisDirection(0.0,0.0,1.0);
      SmSurfOfRevolution * pSurfOfRevo = NULL;
      SER(SmSurfOfRevolution::CreateCanonical(sContext,pCurve,
          sAxisPoint,sAxisDirection,pSurfOfRevo));
      pSurface = pSurfOfRevo;
      SmObjDelete sSU(pSurface);
      SmPoint2d sUVMin = SmPoint2d(0.0,sCrvIvl.GetMin());
      SmPoint2d sUVMax = SmPoint2d(180.0,sCrvIvl.GetMax());
      SmExtent2d sAnalDomain(sUVMin,sUVMax);
      SER(my_test_analytic_surface(pSurface,sAnalDomain));
    }

  if (TRUE)
    { //Surface-of-Extrusion Tests

      // build curve to extrude
      SmTArray<SmPoint3d> sPnts1;
      sPnts1.Add(SmPoint3d(0.0, 0.1, 0.0));
      sPnts1.Add(SmPoint3d(0.3, 0.5, 0.0));
      sPnts1.Add(SmPoint3d(0.5, 0.6, 0.0));
      sPnts1.Add(SmPoint3d(0.6, 0.3, 0.0));
      sPnts1.Add(SmPoint3d(1.0, 0.5, 0.0));
      sPnts1.Add(SmPoint3d(1.3, 0.3, 0.0));
      sPnts1.Add(SmPoint3d(1.5, 0.5, 0.0));
      SmBSplineCurve *pNurb1 = my_create_nurb(sContext,SmPoint3d(0,0,0),sPnts1,3,1,0.5,0);
      SmExtent1d sCrvIvl(0.0,2.0);
      SmBSplineCurve * pCurve = pNurb1;
      SER(pCurve->AdjustSTEPInterval(sCrvIvl));

      // gwc: until the problem with mapping from STEPtoNURBs while fixing the two domains to be
      //      the same is resolved for SmParabola and SmHyperbola - don't use the hyperbola
      //      // build a Hyperbola with a finite interval
      //      SmPoint3d  sOrigin(-1.0,0.0,0.0);
      //      SmVector3d sX     (-1.0,0.0,0.0);
      //      SmVector3d sY     ( 0.0,0.0,1.0);
      //      SmAxis2Placement sPos;
      //      sPos.SetCanonical(sOrigin,sX,sY);
      //      SmHyperbola * pHyperbola = NULL;
      //      SmHyperbola::CreateCanonical(sContext,sPos,0.4,0.5,pHyperbola);
      //      SmExtent1d sCrvIvl(0.0,2.0);
      //      SmBSplineCurve * pCurve = pHyperbola;
      //      SER(pCurve->AdjustSTEPInterval(sCrvIvl));

      // extrude the curve into a surface
      SmVector3d sExtrusionVec(0.0,1.0,0.8);
      SmSurfOfExtrusion * pSurfOfExtru = NULL;
      SER(SmSurfOfExtrusion::CreateCanonical(sContext,
                                             pCurve,
                                             sExtrusionVec,
                                             pSurfOfExtru));
      pSurface = pSurfOfExtru;
      SmObjDelete sSU(pSurface);
      SmPoint2d sUVMin = SmPoint2d(sCrvIvl.GetMin(),-1.0);
      SmPoint2d sUVMax = SmPoint2d(sCrvIvl.GetMax(),3.0);
      SmExtent2d sAnalDomain(sUVMin,sUVMax);
      SER(my_test_analytic_surface(pSurface,sAnalDomain));
    }

  if (TRUE)
    { //Surface-of-Revolution Tests
      SmPoint3d sOrigin(1.0,0.0,0.0);
      SmVector3d sX(1.0,0.0,0.0);
      SmVector3d sY(0.0,0.0,1.0);
      SmAxis2Placement sPos;
      sPos.SetCanonical(sOrigin,sX,sY);
      SmTArray<SmPoint3d> sPnts1;
      sPnts1.Add(SmPoint3d(0,0.1,0));
      sPnts1.Add(SmPoint3d(0.3,0.5,0));
      sPnts1.Add(SmPoint3d(0.5,1,0.2));
      sPnts1.Add(SmPoint3d(0.6,0.3,0.3));
      sPnts1.Add(SmPoint3d(1.0,1.0,0.0));
      sPnts1.Add(SmPoint3d(1.3,0.3,0.1));
      sPnts1.Add(SmPoint3d(1.5,0.5,0));
      SmBSplineCurve *pNurb1 = my_create_nurb(sContext,SmPoint3d(0,0,0),sPnts1,3,1,0.5,0);

      SmPoint3d sAxisPoint(0.0,0.0,0.0);
      SmVector3d sAxisDirection(1.0,0.0,0.0);
      SmSurfOfRevolution * pSurfOfRevo = NULL;
      SER(SmSurfOfRevolution::CreateCanonical(sContext,pNurb1,
          sAxisPoint,sAxisDirection,pSurfOfRevo));
      pSurface = pSurfOfRevo;
      SmObjDelete sSU(pSurface);
      SmExtent1d sCrvIvl = pNurb1->GetNaturalInterval();
      SmPoint2d sUVMin = SmPoint2d(0.0,sCrvIvl.GetMin());
      SmPoint2d sUVMax = SmPoint2d(180.0,sCrvIvl.GetMax());
      SmExtent2d sAnalDomain(sUVMin,sUVMax);
      SER(my_test_analytic_surface(pSurface,sAnalDomain));
    }

  if (TRUE)
    { //Surface-of-Revolution Tests
      SmPoint3d sOrigin(1.0,0.0,0.0);
      SmVector3d sX(1.0,0.0,0.0);
      SmVector3d sY(0.0,0.0,1.0);
      SmAxis2Placement sPos;
      sPos.SetCanonical(sOrigin,sX,sY);
      SmTArray<SmPoint3d> sPnts1;
      sPnts1.Add(SmPoint3d(0,0.1,0));
      sPnts1.Add(SmPoint3d(0.3,0.5,0));
      sPnts1.Add(SmPoint3d(0.5,1,0.2));
      sPnts1.Add(SmPoint3d(0.6,0.3,0.3));
      sPnts1.Add(SmPoint3d(1.0,1.0,0.0));
      sPnts1.Add(SmPoint3d(1.3,0.3,0.1));
      sPnts1.Add(SmPoint3d(1.5,0.5,0));
      SmBSplineCurve *pNurb1 = my_create_nurb(sContext,SmPoint3d(0,0,0),sPnts1,3,1,0.5,0);

      SmPoint3d sAxisPoint(0.0,0.0,0.0);
      SmVector3d sAxisDirection(1.0,0.0,0.0);
      SmSurfOfRevolution * pSurfOfRevo = NULL;
      SER(SmSurfOfRevolution::CreateCanonical(sContext,pNurb1,
          sAxisPoint,sAxisDirection,pSurfOfRevo));
      pSurface = pSurfOfRevo;
      SmObjDelete sSU(pSurface);
      SmExtent1d sCrvIvl = pNurb1->GetNaturalInterval();
      SmPoint2d sUVMin = SmPoint2d(0.0,sCrvIvl.GetMin());
      SmPoint2d sUVMax = SmPoint2d(180.0,sCrvIvl.GetMax());
      SmExtent2d sAnalDomain(sUVMin,sUVMax);
      SER(my_test_analytic_surface(pSurface,sAnalDomain));
    }


  if (TRUE)
    { //Surface-of-Extrusion Tests
      SmPoint3d sOrigin(-1.0,0.0,0.0);
      SmVector3d sX(-1.0,0.0,0.0);
      SmVector3d sY(0.0,0.0,1.0);
      SmAxis2Placement sPos;
      sPos.SetCanonical(sOrigin,sX,sY);

      SmTArray<SmPoint3d> sPnts1;
      sPnts1.Add(SmPoint3d(0,0.1,0));
      sPnts1.Add(SmPoint3d(0.3,0.5,0));
      sPnts1.Add(SmPoint3d(0.5,1,0.2));
      sPnts1.Add(SmPoint3d(0.6,0.3,0.3));
      sPnts1.Add(SmPoint3d(1.0,1.0,0.0));
      sPnts1.Add(SmPoint3d(1.3,0.3,0.1));
      sPnts1.Add(SmPoint3d(1.5,0.5,0));
      SmBSplineCurve *pNurb1 = my_create_nurb(sContext,SmPoint3d(0,0,0),sPnts1,3,1,0.5,0);

      SmPoint3d sAxisPoint(0.0,0.0,0.0);
      SmVector3d sExtrusionVec(0.0,0.0,1.0);
      SmSurfOfExtrusion * pSurfOfExtru = NULL;
      SER(SmSurfOfExtrusion::CreateCanonical(sContext,pNurb1,
          sExtrusionVec,pSurfOfExtru));
      pSurface = pSurfOfExtru;
      SmObjDelete sSU(pSurface);
      SmExtent1d sCrvIvl = pNurb1->GetNaturalInterval();
      SmPoint2d sUVMin = SmPoint2d(sCrvIvl.GetMin(),-1.0);
      SmPoint2d sUVMax = SmPoint2d(sCrvIvl.GetMax(),3.0);
      SmExtent2d sAnalDomain(sUVMin,sUVMax);
      SER(my_test_analytic_surface(pSurface,sAnalDomain));
    }

  return SM_SUCCESS;

} // end my_test_analytic_surfaces

/***********************************************************************
PURPOSE ---

USAGE NOTES ---
***********************************************************************/
PT_EXPORT SmStatus my_test_bsplinesurface_nlib(void)
{
    SmContext sContext;

    SmTArray<SmPoint3d> sPnts;
    my_load_surf1_pts(sPnts);

    if (TRUE) { //AdjustBoundaryCurve
        SmBSplineSurface *pBSS = my_create_nurb_surf(sContext,SmPoint3d(0,0,0),sPnts,6,4,3,3,0.0,0.0,0.0);
        SmObjDelete sSU1(pBSS);
        SmTArray<SmPoint3d> sPnts1;
        sPnts1.Add(SmPoint3d(0,0,-0.6));
        sPnts1.Add(SmPoint3d(1,0,0.4));
        sPnts1.Add(SmPoint3d(2,0,0.3));
        sPnts1.Add(SmPoint3d(3,0,0));
        SmBSplineCurve *pNurb1 = my_create_nurb(sContext,SmPoint3d(0,0,0),sPnts1,3,1,0.5,0);
        SmObjDelete sCU1(pNurb1);
        SER(pBSS->AdjustBoundaryCurve(pNurb1, //const SmBSplineCurve * cpNewBoundaryCurve,
                                      SM_SP_U,//SmSurfParamType eSurfDir,
                                      TRUE,   //SmBoolean bIsNewBdryAtMin,
                                      0.1));  //double dPercentAffected)
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
            smgfx_Erase();
            smgfx_SetLook(1, 2, 0, 0, 1); pBSS->Draw();
            sm_GraphicsLoop();
        }
#endif
    }
    if (TRUE) { //ApproximateOffsetSurface
        SmBSplineSurface *pBSS = my_create_nurb_surf(sContext,SmPoint3d(0,0,0),sPnts,6,4,3,3,0.0,0.0,0.0);
        SmObjDelete sSU(pBSS);
        SmBSplineSurface *pNewSurf = NULL;
        double dTol = 1.0e-2;
        SER(pBSS->ApproximateOffsetSurface
               (sContext,       // const SmContext & crContext,
                0.2,            // double dOffsetDistance,
                dTol,           // double dThisApproxTol3d,
                pNewSurf,       // SmBSplineSurface *& rpNewSurf)
                1));            // chordlength
        SmObjDelete sSU1(pNewSurf);
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
            smgfx_SetLook(1, 2, 0, 1, 0); pNewSurf->Draw();
            sm_GraphicsLoop();
        }
#endif
    }
    if (TRUE) { //ApproximatePoints
        SmBSplineSurface *pNewSurf = NULL;
        double dTol = SM_ZONE_TOL_3D;
        SER(SmBSplineSurface::ApproximatePoints(sContext,//const SmContext & crContext,
                                                sPnts,   //const SmTArray<SmPoint3d> & crPoints,
                                                6,       //ULONG lNumRows,
                                                4,       //ULONG lNumCols,
                                                3,       //ULONG lUDegree,
                                                3,       //ULONG lVDegree,
                                                &dTol,   //double * pdOptTolerance,
                                                pNewSurf));//SmBSplineSurface *& rpNewSurfpNewSurf)
        SmObjDelete sSU1(pNewSurf);
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
            smgfx_SetLook(1, 2, 0, 0, 1); pNewSurf->Draw();
            sm_GraphicsLoop();
        }
#endif
    }
    if (TRUE) { //ApproximateRandomPoints
        SmTArray<SmPoint3d> sPnts1;
        my_load_surf3_pts(sPnts1);
        SmBSplineSurface *pNewSurf = NULL;
        SER(SmBSplineSurface::ApproximateRandomPoints(
                                                sContext,//const SmContext & crContext,
                                                sPnts1,   //const SmTArray<SmPoint3d> & crPoints,
                                                6,       //ULONG lMaxUCtrlPnts,
                                                6,       //ULONG lMaxVCtrlPnts,
                                                3,       //ULONG lUDegree,
                                                3,       //ULONG lVDegree,
                                                NULL,    //const SmTArray<SmBSplineCurve*> * cpOptBdryCurves,
                                                1,       //ULONG lComplexity,
                                                pNewSurf));//SmBSplineSurface *& rpNewSurfpNewSurf)
        SmObjDelete sSU1(pNewSurf);
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
            smgfx_SetColor(1, 0, 0);
            for (ULONG i = 0; i < sPnts1.GetSize(); i++) {
                sPnts1[i].Draw();
            }
            pNewSurf->Draw();
            sm_GraphicsLoop();
        }
#endif
    }
    if (TRUE) { //ApproximateRationalSurface
        SmTArray<SmPoint3d> sPnts1;
        sPnts1.Add(SmPoint3d(0,0.1,0));
        sPnts1.Add(SmPoint3d(0.3,0.5,0));
        sPnts1.Add(SmPoint3d(0.5,1,0.0));
        sPnts1.Add(SmPoint3d(0.6,0.3,0.0));
        sPnts1.Add(SmPoint3d(1.0,1.0,0.0));
        sPnts1.Add(SmPoint3d(1.3,0.3,0.0));
        sPnts1.Add(SmPoint3d(1.5,0.5,0));
        SmBSplineCurve *pNurb1 = my_create_nurb(sContext,SmPoint3d(0,0,0),sPnts1,3,1,0.5,0);
        SmObjDelete sCU1(pNurb1);
        SmBSplineSurface *pBSS = NULL ;
        SER(SmBSplineSurface::CreateSurfOfRevolution(sContext,pNurb1,SmPoint3d(0,2,0),SmVector3d(1,1,0),
            360.0,pBSS));
        SmObjDelete sSU(pBSS);
        SmBSplineSurface *pNewSurf = NULL;
        double dTol = 1.0e-3;
        SER(pBSS->ApproximateRationalSurface(sContext,  //const SmContext & crContext,
                                             dTol,      //double dThisApproxTol3d,
                                             pNewSurf));//SmBSplineSurface *& rpNewSurf)
        SmObjDelete sSU1(pNewSurf);
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
            smgfx_SetColor(0, 0, 0);
            pBSS->Draw();
            sm_GraphicsLoop();
            smgfx_SetColor(1, 0, 0);
            pNewSurf->Draw();
            sm_GraphicsLoop();
        }
#endif
    }
    if (TRUE) { //ApproximateSubSurface
        SmBSplineSurface *pBSS = my_create_nurb_surf(sContext,SmPoint3d(0,0,0),sPnts,6,4,3,3,0.0,0.0,0.0);
        SmObjDelete sSU(pBSS);
        SmExtent2d sUVDomain = pBSS->GetNaturalUVDomain();

        SmPoint3d sPnt1(0.25,0.5,0.0);
        SmPoint3d sPnt2(0.5,0.25,0.0);
        SmPoint3d sPnt3(0.75,0.5,0.0);
        SmPoint3d sPnt4(0.5,0.75,0.0);
        SmTArray<SmBSplineCurve*> s3DCurves;
        SmTArray<SmBSplineCurve*> sUVCurves;
        double dTol = 1.0e-3;
        double dDist;

        SmBSplineCurve *pUVCurve1 = NULL;
        SER(SmBSplineCurve::CreateLineSegment(sContext,2,sPnt1,sPnt2,pUVCurve1));
        NER(pUVCurve1);
        sUVCurves.Add(pUVCurve1);
        SmBSplineCurve * p3DCurve1 = NULL;
        SER(pBSS->LiftCurve(sContext,sUVDomain,*pUVCurve1,
            pUVCurve1->GetNaturalInterval(),dTol,dDist,p3DCurve1));
        NER(p3DCurve1);
        s3DCurves.Add(p3DCurve1);

        SmBSplineCurve *pUVCurve2 = NULL;
        SER(SmBSplineCurve::CreateLineSegment(sContext,2,sPnt4,sPnt3,pUVCurve2));
        NER(pUVCurve2);
        sUVCurves.Add(pUVCurve2);
        SmBSplineCurve * p3DCurve2 = NULL;
        SER(pBSS->LiftCurve(sContext,sUVDomain,*pUVCurve2,
            pUVCurve2->GetNaturalInterval(),dTol,dDist,p3DCurve2));
        NER(p3DCurve2);
        s3DCurves.Add(p3DCurve2);

        SmBSplineCurve *pUVCurve3 = NULL;
        SER(SmBSplineCurve::CreateLineSegment(sContext,2,sPnt1,sPnt4,pUVCurve3));
        NER(pUVCurve3);
        sUVCurves.Add(pUVCurve3);
        SmBSplineCurve * p3DCurve3 = NULL;
        SER(pBSS->LiftCurve(sContext,sUVDomain,*pUVCurve3,
            pUVCurve3->GetNaturalInterval(),dTol,dDist,p3DCurve3));
        NER(p3DCurve3);
        s3DCurves.Add(p3DCurve3);

        SmBSplineCurve *pUVCurve4 = NULL;
        SER(SmBSplineCurve::CreateLineSegment(sContext,2,sPnt2,sPnt3,pUVCurve4));
        NER(pUVCurve4);
        sUVCurves.Add(pUVCurve4);
        SmBSplineCurve * p3DCurve4 = NULL;
        SER(pBSS->LiftCurve(sContext,sUVDomain,*pUVCurve4,
            pUVCurve4->GetNaturalInterval(),dTol,dDist,p3DCurve4));
        NER(p3DCurve4);
        s3DCurves.Add(p3DCurve4);

        SmBSplineSurface *pNewSurf = NULL;
        SER(pBSS->ApproximateSubSurface(sContext,  //const SmContext & crContext,
                                        s3DCurves, //const SmTArray<SmBSplineCurve*> & cr3DCurves,
                                        sUVCurves, //const SmTArray<SmBSplineCurve*> & crUVCurves,
                                        pNewSurf));//SmBSplineSurface *& rpNewSurf)
        SmObjDelete sSU1(pNewSurf);
        SmObjsDelete<SmBSplineCurve*> sClean3D( &s3DCurves ), sCleanUV( &sUVCurves );
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
            sm_GraphicsLoop();
            smgfx_SetColor(1, 0, 0);
            pNewSurf->Draw();
            sm_GraphicsLoop();
        }
#endif
    }
    if (TRUE) { //CalculateCrossBoundaryDerivative
        SmBSplineSurface *pBSS = my_create_nurb_surf(sContext,SmPoint3d(0,0,0),sPnts,6,4,3,3,0.0,0.0,0.0);
        SmObjDelete sSU1(pBSS);
        SmBSplineCurve * pDerivCurve = NULL;
        SER(pBSS->CalculateCrossBoundaryDerivative(sContext, //const SmContext & crContext,
                                                   SM_SP_U,  //SmSurfParamType eSurfDir,
                                                   TRUE,     //SmBoolean bIsAtMin,
                                                   pDerivCurve));//SmBSplineCurve *& rpDerivCurve)
        SmObjDelete sCU1(pDerivCurve);
    }
    if (TRUE) { //CreateGordenSurface
        SmBSplineSurface *pBSS = my_create_nurb_surf(sContext,SmPoint3d(0,0,0),sPnts,6,4,3,3,0.0,0.0,0.0);
        SmObjDelete sSU(pBSS);

        SmTArray<SmBSplineCurve*> sUCurves;
        SmTArray<SmBSplineCurve*> sVCurves;

        double dTol = 1.0e-4;
        for (ULONG i=0; i<5; i++) {
            double dU = 0.25*i;
            SmBSplineCurve *pIsoCrv = NULL;
            SER(pBSS->CreateIsoParametricCurve(sContext,SM_SP_U,dU,
                dTol,pIsoCrv)); NER(pIsoCrv);
            sUCurves.Add(pIsoCrv);
        }

        for (ULONG j=0; j<6; j++) {
            double dV = 0.2*j;
            SmBSplineCurve *pIsoCrv = NULL;
            SER(pBSS->CreateIsoParametricCurve(sContext,SM_SP_V,dV,
                dTol,pIsoCrv)); NER(pIsoCrv);
            sVCurves.Add(pIsoCrv);
        }

        SmBSplineSurface *pNewSurf = NULL;
        SER(SmBSplineSurface::CreateGordonSurface(sContext,  //const SmContext & crContext,
                                                  sUCurves,  //const SmTArray<SmBSplineCurve*> & crUCurves,
                                                  sVCurves,  //const SmTArray<SmBSplineCurve*> & crVCurves,
                                                  pNewSurf));//SmBSplineSurface *& rpNewSurf)
        SmObjDelete sSU1(pNewSurf);
        SmObjsDelete<SmBSplineCurve*> sCleanU( &sUCurves ), sCleanV( &sVCurves );
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
            sm_GraphicsLoop();
            smgfx_SetColor(1, 0, 0);
            pNewSurf->Draw();
            sm_GraphicsLoop();
        }
#endif
    }
    if (TRUE) { //CreateSweepSurface
        SmTArray<SmPoint3d> sPnts1;
        sPnts1.Add(SmPoint3d(0,0.0,-1.0));
        sPnts1.Add(SmPoint3d(0,0.5,2.0));
        sPnts1.Add(SmPoint3d(0,1,-1));
        sPnts1.Add(SmPoint3d(0,1.5,0));
        sPnts1.Add(SmPoint3d(0,2,2));
        sPnts1.Add(SmPoint3d(0,2.5,-1.2));
        sPnts1.Add(SmPoint3d(0,3.0,0));
        SmBSplineCurve * pSectionCurve = my_create_nurb(sContext,SmPoint3d(0,0,0),sPnts1,3,1.0,0.0,0.0);

        SmTArray<SmPoint3d> sPnts2;
        sPnts2.Add(SmPoint3d(0.0,0.0,-1.0));
        sPnts2.Add(SmPoint3d(1.0,0.3,0.5));
        sPnts2.Add(SmPoint3d(2.0,0.0,-1.0));
        sPnts2.Add(SmPoint3d(3.0,-0.5,0.5));
        SmBSplineCurve * pPathCurve = my_create_nurb(sContext,SmPoint3d(0,0,0),sPnts2,3,1.0,0.0,0.0);
        SmBSplineSurface *pNewSurf = NULL;
        double dTol = 1.0e-4;
        SER(SmBSplineSurface::CreateSweepSurface(sContext,  //const SmContext & crContext,
                                                 pSectionCurve,//const SmBSplineCurve * cpSectionCurve,
                                                 pPathCurve,//const SmBSplineCurve * cpTrajectoryCurves,
                                                 NULL,
                                                 TRUE,      //SmBoolean bDoTranslationalSweep,
                                                 dTol,      //double dThisApproxTol3d,
                                                 pNewSurf));//SmBSplineSurface *& rpNewSurfpNewSurf)
        SmObjDelete sSU1( pNewSurf ), sCleanSC( pSectionCurve ), sCleanPC( pPathCurve );
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
            smgfx_SetColor(0, 0, 0);
            pNewSurf->Draw();
            sm_GraphicsLoop();
        }
#endif
    }
    if (TRUE) { //CreateSwungSurface
        SmBSplineCurve *pSectionCurve = my_create_line(sContext,SmPoint3d(1.0,0.0,0.0),SmPoint3d(0.0,0.0,1.0),0.2,0,0.3);
        SmBSplineCurve *pPathCurve = my_create_line(sContext,SmPoint3d(1.0,0.0,0.0),SmPoint3d(0.0,1.0,0.0),0.2,0,0.3);
        SmObjDelete sCleanup(pSectionCurve);
        SmObjDelete sCleanup2(pPathCurve);
        SmBSplineSurface *pNewSurf = NULL;
        double dScale = 1.5;
        SER(SmBSplineSurface::CreateSwungSurface(sContext,  //const SmContext & crContext,
                                                 pSectionCurve,//const SmBSplineCurve * cpSectionCurve,
                                                 pPathCurve,//const SmBSplineCurve * cpTrajectoryCurves,
                                                 dScale,    //double dScaleFactor,
                                                 pNewSurf));//SmBSplineSurface *& rpNewSurfpNewSurf)
        SmObjDelete sSU1(pNewSurf);
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
            smgfx_SetColor(0, 0, 0);
            pNewSurf->Draw();
            sm_GraphicsLoop();
        }
#endif
    }
    if (TRUE) { //DegreeElevate
        SmBSplineSurface *pBSS = my_create_nurb_surf(sContext,SmPoint3d(0,0,0),sPnts,6,4,2,3,0.0,0.0,0.0);
        SmObjDelete sSU1(pBSS);
        SER(pBSS->DegreeElevate(SM_SP_U, //SmSurfParamType eDirectionToElevate,
                                3));     //ULONG lNewDegree
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
            smgfx_SetColor(1, 0, 0);
            pBSS->Draw();
            sm_GraphicsLoop();
        }
#endif
    }
    if (TRUE) { //DegreeReduction
        SmBSplineSurface *pBSS = my_create_nurb_surf(sContext,SmPoint3d(0,0,0),sPnts,6,4,3,3,0.0,0.0,0.0);
        SmObjDelete sSU1(pBSS);
        double dTol = 1.0e-4;
        SER(pBSS->DegreeReduction(dTol,     //double dTolerance,
                                  FALSE,    //SmBoolean bMaxReduce,
                                  SM_SP_U));//SmSurfParamType eDirectionToReduce)
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
            smgfx_SetColor(1, 0, 0);
            pBSS->Draw();
            pBSS->Dump(TRUE);
            sm_GraphicsLoop();
        }
#endif
    }
    if (TRUE) { //GetMeasures
        SmBSplineSurface *pBSS = my_create_nurb_surf(sContext,SmPoint3d(0,0,0),sPnts,6,4,3,3,0.0,0.0,0.0);
        SmObjDelete sSU1(pBSS);
        double dAverageLengthU;
        double dAverageLengthV;
        double dEstimatedAreaBound;
        SER(pBSS->GetMeasures(dAverageLengthU, //double & rdAverageLengthU,
                              dAverageLengthV, //double & rdAverageLengthV,
                              dEstimatedAreaBound));//double & rdEstimatedAreaBound
    }
    if (TRUE) { //InsertOneKnot
        SmBSplineSurface *pBSS = my_create_nurb_surf(sContext,SmPoint3d(0,0,0),sPnts,6,4,3,3,0.0,0.0,0.0);
        SmObjDelete sSU1(pBSS);
        pBSS->Dump(TRUE);
        double dKnot = 0.456;
        SER(pBSS->InsertOneKnot(dKnot,    //double dKnot,
                                2,        //ULONG lNumKnotInsertions,
                                SM_SP_V));//SmSurfParamType eSurfParam)
        pBSS->Dump(TRUE);
    }
    if (TRUE) { //InterpolatePoints
        SmBSplineSurface *pNewSurf = NULL;
        SER(SmBSplineSurface::InterpolatePoints(sContext,//const SmContext & crContext,
                                                sPnts,   //const SmTArray<SmPoint3d> & crPoints,
                                                6,       //ULONG lNumRows,
                                                4,       //ULONG lNumCols,
                                                3,       //ULONG lUDegree,
                                                3,       //ULONG lVDegree,
                                                NULL,    //const SmTArray<SmBSplineCurve*> * cpOptBdryCurves,
                                                NULL,    //const SmTArray<double> * cpOptUParams,
                                                NULL,    //const SmTArray<double> * cpOptVParams,
                                                SM_IT_CHORDLENGTH,//SmInterpolationType eParameterization,
                                                pNewSurf));//SmBSplineSurface *& rpNewSurfpNewSurf)
        SmObjDelete sSU1(pNewSurf);
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
            smgfx_SetColor(1, 0, 0);
            pNewSurf->Draw();
            sm_GraphicsLoop();
        }
#endif
    }
    if (TRUE) { //RefineSurface
        SmBSplineSurface *pBSS = my_create_nurb_surf(sContext,SmPoint3d(0,0,0),sPnts,6,4,3,3,0.0,0.0,0.0);
        SmObjDelete sSU1(pBSS);
        pBSS->Dump(TRUE);
        SmTArray<double> sNewKnots;
        sNewKnots.Add(0.25);
        sNewKnots.Add(0.35);
        sNewKnots.Add(0.45);
        sNewKnots.Add(0.55);
        sNewKnots.Add(0.65);
        sNewKnots.Add(0.75);
        SER(pBSS->RefineSurface(sNewKnots,//const SmTArray<double> & crNewKnots
                                SM_SP_V));//SmSurfParamType eSurfParam)
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
            smgfx_SetColor(1, 0, 0);
            pBSS->Draw();
            pBSS->Dump(TRUE);
            sm_GraphicsLoop();
        }
#endif
    }
    if (TRUE) { //RemoveKnots
        SmBSplineSurface *pBSS = my_create_nurb_surf(sContext,SmPoint3d(0,0,0),sPnts,6,4,3,3,0.0,0.0,0.0);
        SmObjDelete sSU1(pBSS);
        SmTArray<double> sNewKnots;
        sNewKnots.Add(0.25);
        sNewKnots.Add(0.35);
        sNewKnots.Add(0.45);
        sNewKnots.Add(0.55);
        sNewKnots.Add(0.65);
        sNewKnots.Add(0.75);
        SER(pBSS->RefineSurface(sNewKnots,//const SmTArray<double> & crNewKnots
                                SM_SP_V));//SmSurfParamType eSurfParam)
        pBSS->Dump(TRUE);
        double dTol = 1.0e-4;
        SmTArray<double> sKeptVKnots;
        sKeptVKnots.Add(0.0);
        sKeptVKnots.Add(0.55);
        sKeptVKnots.Add(1.0);
        SER(pBSS->RemoveKnots(dTol,  //double dThisApproxTol3d,
                              FALSE, //SmBoolean bRemoveUKnots,
                              TRUE,  //SmBoolean bRemoveVKnots,
                              NULL,  //const SmTArray<double> * cpOptKeptUKnots = NULL,
                              &sKeptVKnots));//const SmTArray<double> * cpOptKeptVKnots = NULL
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
            smgfx_SetColor(1, 0, 0);
            pBSS->Dump(TRUE);
            pBSS->Draw();
            sm_GraphicsLoop();
        }
#endif
    }
    if (TRUE) { //RemoveOneKnot
        SmBSplineSurface *pBSS = my_create_nurb_surf(sContext,SmPoint3d(0,0,0),sPnts,6,4,3,3,0.0,0.0,0.0);
        SmObjDelete sSU1(pBSS);
        SmTArray<double> sNewKnots;
        sNewKnots.Add(0.25);
        sNewKnots.Add(0.35);
        sNewKnots.Add(0.45);
        sNewKnots.Add(0.55);
        sNewKnots.Add(0.65);
        sNewKnots.Add(0.75);
        SER(pBSS->RefineSurface(sNewKnots,//const SmTArray<double> & crNewKnots
                                SM_SP_V));//SmSurfParamType eSurfParam)
        pBSS->Dump(TRUE);
        double dTol = 1.0e-4;
        double dKnot = 0.45;
        ULONG  lNumKnotsRemoved;
        SER(pBSS->RemoveOneKnot(dKnot,  //double dKnot,
                                3,      //ULONG lNumKnotsRemoval,
                                SM_SP_V,//SmSurfParamType eSurfParam,
                                dTol,   //double dThisApproxTol3d,
                                lNumKnotsRemoved));//ULONG & rlNumKnotsRemoved)
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
            smgfx_SetColor(1, 0, 0);
            pBSS->Dump(TRUE);
            pBSS->Draw();
            sm_GraphicsLoop();
        }
#endif
    }



    return SM_SUCCESS;

} // end my_test_bsplinesurface_nlib
