// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmSurfaceCache.cpp
* PURPOSE: Source file for SmSurfaceCache methods.
**********************************************************************/

#include "StdAfx.h"

#include <SmBSplineSurface.h>
#include <SmSurfaceCache.h>
#include <nurbs.h>
#include <SmNurbsSrf.h>
#include <SmNurbsCrv.h>       
#include <SmGraphicsExtern.h>
#include <SmTArray.h>
#include <SmGeomUtility.h>
#include <SmTree.h>
#include <SmPolarBox.h>
#include <SmIsoCurve.h>
#include <SmCubicBezierSurface.h>
#include <SmGraphicsOutput.h>                // smgfx_OutputColor()
                                         // smgfx_OutputObjectColor()
#include <SmAssertArray.h>
#include <SmPlane.h>
#include <SmPoly.h>

#include <SmFace.h>
// Remove Composite
//  #include <SmCFace.h>
#include <SmVertexuse.h>                 // SmVertexList::Dump()
#include <SmEdgeuse.h>                   // SmEdgeuseList::Dump

#include <SmTess.h>

#ifdef SM_DEBUG_CODE
#include <SmTrimSrfCache.h>
#include <SmBrep.h>
#include <SmPlane.h>


#endif

#if 0
/********************************************************************
PURPOSE:

NOTES: UNUSED
*********************************************************************/
static const SmSurface & sm_GetOriginalSurface
  (const SmTreeNode & crSurfaceNode)
{
    SmTree *pTree = crSurfaceNode.m_pTree;
    if (!pTree->GetOwner()->IsKindOf(SmSurfaceCache_TYPE)) {
        SE(SM_ERR);  // should never happen
    }
    SmSurfaceCache *pSC = (SmSurfaceCache*)pTree->GetOwner();
    if (pSC == NULL) {
        SE(SM_ERR); // should never happen
    }
    return pSC->GetSurface();

} // end sm_GetOriginalSurface
#endif

// Implementation of the member functions of SmPatchBoundaryBoundingPlanes

/********************************************************************
PURPOSE: First, a convenience tool which converts 
            SurfParamType/MinMax to the arrary index 
            (see pic in .h for allocation of array slots).

NOTES:
*********************************************************************/
static ULONG sm_SurfparamToIndex
  (SmSurfParamType eSurfParamArg, 
   ULONG nMinMaxArg)
{
    if (eSurfParamArg == SM_SP_V) {
        if (nMinMaxArg == 0)
            return 0;
        else 
            return 2;
    }
    // else // if (eSurfParamArg == SM_SP_U) 
    if (nMinMaxArg == 0)
        return 3;
    return 1;

} // end sm_SurfparamToIndex

/********************************************************************
PURPOSE:

NOTES:
*********************************************************************/
SmPatchBoundaryBoundingPlanes::SmPatchBoundaryBoundingPlanes
  (const SmSurface* pSurfaceArg,
   const SmExtent2d& rsUVDomainArg) 
 : m_pSurface(pSurfaceArg),
   m_sUVDomain(rsUVDomainArg),
   m_bPlanesComputed(FALSE)
{

} // end SmPatchBoundaryBoundingPlanes::SmPatchBoundaryBoundingPlanes constructor

/********************************************************************
PURPOSE:

NOTES:
*********************************************************************/
SmPatchBoundaryBoundingPlanes::~SmPatchBoundaryBoundingPlanes()
{

} // end SmPatchBoundaryBoundingPlanes::~SmPatchBoundaryBoundingPlanes destructor


/********************************************************************
PURPOSE: Draw Bounding Plane

NOTES:
*********************************************************************/
SmDisplayList * SmPatchBoundaryBoundingPlanes::Draw
  ()
 const
{  
  SmDisplayList *pRtn = NULL ;

#ifdef SM_GFX_CODE
  // no work - no planes computed
  if(m_bPlanesComputed == FALSE) return(NULL) ;

  // local - surface bounding box 
  SmExtent3d sBBox ;
  m_pSurface->CalculateBoundingBox(m_sUVDomain, &sBBox) ;
  double dScale = sBBox.GetSize().Length() ;

  // start DisplayList (or use currently open one)
  SmVector3d sColor = smgfx_GetOutputColor() ;
  smgfx_Open(smgfx_GetRuleColor(m_pSurface));

  // for every bounding plane
  for(ULONG ii=0;ii<4;ii++)
    {
      // when ith 3d boundary curve is planar and bounds the surface
      if(   m_bBoundaryCurveDegenerate[ii] == FALSE
         && m_bBoundaryCurvePlanar[ii]     == TRUE)
        {
          // create temporary Finite Plane from the BBox/infinite Plane intersection
          SmSurface *pPlane = NULL ;
          SmSurface::CreatePlaneFromBBox(*m_pSurface->GetContext(),
                                         sBBox,
                                         m_vBasePoint[ii],
                                         m_vNormalVector[ii],
                                         pPlane);
          SmObjDelete CleanUp(pPlane);

          // when pPlane was constructed - draw it
          if(pPlane) 
            { 
              // Draw the Plane with crosshatching
              pPlane->DrawUV(6,6) ; 

              // draw midPoint with Fat SurfaceNormal Vector
              double dLineWidth = smgfx_SetLineWidth(4.0*smgfx_GetLineWidth()) ; 
              SmVector3d sNormVec = dScale * m_vNormalVector[ii] ;
              sNormVec.Draw(&m_vBasePoint[ii]) ;
              smgfx_SetLineWidth(dLineWidth) ;

            } // end pPlane existence check

        } // end 3d boundary curve is planar and bounds the surface check

    } // end iter every bounding plane

  // end DisplayList
  pRtn = smgfx_Close() ;

#endif // SM_GFX_CODE
  return(pRtn) ;

} // end SmPatchBoundaryBoundingPlanes::Draw

/*******************************************************************//*******
* Given the plane of a planar boundary curve, this function checks if this 
* plane is a separating halfspace for the surface patch. It also checks
* if the normal of the plane points 'off the material side'.
*
* Method: the nodes of the control polygon are checked.
*****************************************************************************/
static SmStatus sm_CheckBoundingPlaneSeparatingHalfspace
 (SmBSplineSurface * pBSS, 
  const SmPoint3d  & rsPlaneBasePoint,
  const SmVector3d & rsPlaneNormal,
  SmBoolean        & rbIsSeparatingHalfspace,
  SmBoolean        & rbInvertNormal)
{
    SmTArray<SmPoint3d> sControlPoints;
    SmBSplineSurfaceForm eSurfaceForm;
    SmKnotType eKnotType;
    SmTArray<double> sWeights;
    ULONG nUDegree;
    ULONG nVDegree;
    SmTArray<ULONG> sUKnotMult, sVKnotMult;
    SmTArray<double> sUKnots, sVKnots;

    ULONG nControlPoints = pBSS->GetNumberControlPoints(SM_SP_U) * 
                           pBSS->GetNumberControlPoints(SM_SP_V);

    SER(pBSS->GetCanonical(nUDegree,nVDegree,sControlPoints,
        eSurfaceForm,sUKnotMult,sVKnotMult,sUKnots,sVKnots,
        eKnotType,sWeights));

    if (nControlPoints != sControlPoints.GetSize())  { SER(SM_ERR); }

    int lSign = 0;

    for(ULONG ii=0; ii<nControlPoints; ii++) {
        const SmPoint3d& rsCP = sControlPoints[ii];

        double dSignedDist = rsPlaneNormal.Dot( rsCP - rsPlaneBasePoint );

        if (smos_Fabs(dSignedDist) < SM_EFF_ZERO_SQRT )
            continue;

        // If there is a sign difference: the plane is not a 
        // separating halfspace
        if (   (dSignedDist > 0.0 && lSign == -1) 
            || (dSignedDist < 0.0 && lSign ==  1) )  {
            rbIsSeparatingHalfspace = FALSE;
            return SM_SUCCESS;
        }

        if (dSignedDist > 0.0)
           lSign = 1;
        else 
           lSign = -1;
    }
    
    // All points are expected to be in the 'material side' of the plane
    // (the normal pointing out of it):
    rbInvertNormal = FALSE;
    if (lSign == 1)
        rbInvertNormal = TRUE;

    rbIsSeparatingHalfspace = TRUE;
    return SM_SUCCESS;


} // end sm_CheckBoundingPlaneSeparatingHalfspace

/*******************************************************************//******
* Given 3 points (samples from a boundary curve, this function creates their
* plane unless the points are (partially) collinear. 
*
* The tolerance for edge degeneracy is given on input, but mind 2 the angular
* tolerances (dAngleTol, SM_EFF_ZERO_SQRT) used within.
****************************************************************************/
static SmStatus sm_ComputePlaneFrom3Points
  (const SmPoint3d& sP1Arg, 
   const SmPoint3d& sP2Arg, 
   const SmPoint3d& sP3Arg, 
   SmPoint3d& rsPlaneBaseArg, 
   SmVector3d& rsPlaneNormalArg,
   double dTolArg,
   SmBoolean& rbPlaneExistsArg,
   SmBoolean& rb3PointsCollinearArg)
{
    rbPlaneExistsArg = FALSE;
    rb3PointsCollinearArg = FALSE;

    // Avoiding insanity: 
    SmVector3d sP12V = sP1Arg - sP2Arg;
    SmVector3d sP23V = sP2Arg - sP3Arg;
    SmVector3d sP31V = sP3Arg - sP1Arg;
   
    if ( sP12V.Length() < dTolArg ||
         sP23V.Length() < dTolArg ||
         sP31V.Length() < dTolArg ) {
        return SM_SUCCESS;
    }

    sP12V.Unitize();
    sP23V.Unitize();
    sP31V.Unitize();

    double dAngleTol = 0.01;

    SmVector3d sP123( sP12V * sP23V);         
    SmVector3d sP231( sP23V * sP31V);         
    SmVector3d sP312( sP31V * sP12V);         
    
    double dSin12 = sP123.Length();
    double dSin23 = sP231.Length();
    double dSin31 = sP312.Length();
  
    if(dSin12 < dAngleTol || dSin23 < dAngleTol || dSin31 < dAngleTol)
    {

      if(dSin12 < SM_EFF_ZERO_SQRT &&
         dSin23 < SM_EFF_ZERO_SQRT &&
         dSin31 < SM_EFF_ZERO_SQRT)
      {
        rb3PointsCollinearArg = TRUE;
      }
      return SM_SUCCESS;
    }

    // Check for safety (should be OK):
    if (sP123.Dot(sP231) < 0.0 ||
        sP231.Dot(sP312) < 0.0 ||
        sP312.Dot(sP123) < 0.) { SER(SM_ERR); }

    // If made it here, there exists a plane:
    rbPlaneExistsArg = TRUE;
    rsPlaneBaseArg =   (sP1Arg  + sP2Arg  + sP3Arg )  / 3.0;
    rsPlaneNormalArg = (sP123   + sP231   + sP312 )   / 3.0;
    rsPlaneNormalArg.Unitize();
    return SM_SUCCESS;

} // end sm_ComputePlaneFrom3Points


/*******************************************************************//*******
PURPOSE: This is a convenience tool, which helps avoid code repetition.
            It returns the boundary curve, which corresponds to the side
            of the patch given by nIndexArg (see picture in the header file).
            
NOTES: 
****************************************************************************/
SmStatus SmPatchBoundaryBoundingPlanes::CreateBoundaryCurveForIndex
  (ULONG             nIndex,     // in : 0-3
   SmBSplineCurve *& pBSCArg)    // out: newly constructed curve
{
  // get boundary curve isoParameter and direction
  SmSurfParamType eSPT ;
  double          dParam ;
  SmPoint2d       sDomainCorner =  (nIndex == 0 || nIndex == 3)
                                  ? m_sUVDomain.GetMin()
                                  : m_sUVDomain.GetMax();
  if (nIndex == 0 ||  nIndex == 2) { dParam = sDomainCorner.y;
                                     eSPT = SM_SP_V;
                                   }
  else                             { dParam = sDomainCorner.x;
                                     eSPT = SM_SP_U;
                                   }

  SmBSplineSurface* pBSS = SM_CAST_PTR(SmBSplineSurface, m_pSurface);
  NER(pBSS);

  // make the curve
  SmBSplineCurve* pBSC;
  SER(pBSS->CreateIsoParametricCurve(*pBSS->GetContext(),
                                     eSPT,
                                     dParam,
                                     SM_BIG_DOUBLE, // 3D Tol not used
                                     pBSC));
  NER(pBSC);
  SM_DUMP_AND_ASSERT2_VALID(pBSC) ;

  // all done - set output
  pBSCArg = pBSC;
  return SM_SUCCESS;

} // end SmPatchBoundaryBoundingPlanes::CreateBoundaryCurveForIndex


/*******************************************************************//*******
PURPOSE: This is a convenience tool, to get the UVEndPoints
            for a BoundaryCurve
            
NOTES: 
****************************************************************************/
SmStatus SmPatchBoundaryBoundingPlanes::GetBoundaryUVEndPoints
  (ULONG      nIndex,     // in : 0-3
   SmPoint2d &rMin,       // in : Start UV EndPoint
   SmPoint2d &rMax)       // in : End UV EndPoint
{
  // get boundary curve isoParameter and direction
  double    dParam ;

  // pick a corner on the isoParam curve
  SmPoint2d sDomainCorner =  (nIndex == 0 || nIndex == 3)
                            ? m_sUVDomain.GetMin()
                            : m_sUVDomain.GetMax();

  // switch to set UVEndPoints
  if (nIndex == 0 ||  nIndex == 2) 
    { // eSPT = SM_SP_V;
      dParam = sDomainCorner.y;
      rMin.Set(m_sUVDomain.GetMin().x, dParam);
      rMax.Set(m_sUVDomain.GetMax().x, dParam);
    }
  else                             
    { // eSPT = SM_SP_U; 
      dParam = sDomainCorner.x;
      rMin.Set(dParam, m_sUVDomain.GetMin().y);
      rMax.Set(dParam, m_sUVDomain.GetMax().y);
    }

  // all done 
  return SM_SUCCESS;

} // end SmPatchBoundaryBoundingPlanes::GetBoundaryUVEndPoints

/*******************************************************************//**
PURPOSE: Protected member to compute the bounding planes of the patch 
            boundary, if possible.
            
NOTES: The plane normals, if they exist, point OFF the surface
                (the surface points are on the 'material side' of the plane,
                 the plane normal pointing OFF the material side).
***********************************************************************/
SmStatus SmPatchBoundaryBoundingPlanes::ComputeBoundingPlanes
  ()
{
  double dTolerance = 0.0001;
  // An arbitrary number. The following may be somewhat better, although
  // best would be if we knew the size of the world.

  // The cache is unfortunately not available here
  // SmTree *pTree     = GetTree(); NER(pTree);
  // SmTreeNode *pRoot = pTree->GetTopNode();
  // SmExtent3d sBBox  = pRoot->m_sBBox;
  // double dSurfSize  = sBBox.GetSize();
  // dTolerance        = SM_MAX(dTolerance, dTolerance * dSurfSize);

  // local
  SmBSplineSurface* pBS = SM_CAST_PTR(SmBSplineSurface, m_pSurface);
  NER(pBS);

  // for every boundary
  for (ULONG ii=0; ii<4; ii++) 
    {
      // create a 3D Bspline isoParameter boundary curve
      SmBSplineCurve* pBSP;
      SER(CreateBoundaryCurveForIndex(ii, pBSP));
      SmObjDelete sCleanup(pBSP);
      
      // init planar and degenerate flags to FALSE
      m_bBoundaryCurveDegenerate[ii] = FALSE;
      m_bBoundaryCurvePlanar[ii]     = FALSE;

      // test 3D boundary curve for degeneracy
      if (pBSP->IsDegenerate()) 
        {
          m_bBoundaryCurveDegenerate[ii] = TRUE;
          continue;
        }

      // eval 3D boundary curve beg, mid, and end pts
      SmExtent1d sParametricDomain( pBSP->GetNaturalInterval() );
      SmPoint3d sP1, sP2, sP3, sP12, sP23;
      SER(pBSP->EvaluatePoint(sParametricDomain.Evaluate(0.0),  sP1));        
      SER(pBSP->EvaluatePoint(sParametricDomain.Evaluate(0.25), sP12));        
      SER(pBSP->EvaluatePoint(sParametricDomain.Evaluate(0.5),  sP2));        
      SER(pBSP->EvaluatePoint(sParametricDomain.Evaluate(0.75), sP23));        
      SER(pBSP->EvaluatePoint(sParametricDomain.Evaluate(1.0),  sP3));        

      // compute plane from those 3 points
      SmPoint3d sPlaneBase; SmVector3d sPlaneNormal;
      SmBoolean b3PointsCollinear;
      SmBoolean bPlaneExists;
      SER(sm_ComputePlaneFrom3Points(sP1, sP2, sP3, 
                                        sPlaneBase, sPlaneNormal,
                                        dTolerance,
                                        bPlaneExists,
                                        b3PointsCollinear));

      // The plane may not exist because of some of the edges of the triangle
      // are short, or because of collinearity of the points.
      // In case of collinearity, a plane may still be created as:
      //   Plane_normal = Cross_product(surface_normal,bndry_tangent)
      //     for any point on the boundary curve.

      if (b3PointsCollinear) 
        { 
          sPlaneBase = (sP1 + sP2 + sP3) / 3.0;

          SmBoolean bUFromLeft = TRUE, bVFromLeft = TRUE; 
          if (ii == 0 || ii == 3) 
            {
              bUFromLeft = bVFromLeft = FALSE;
            }

          SmPoint2d sDomainCorner(m_sUVDomain.GetMax());
          if (ii == 0 || ii == 3) 
              sDomainCorner = m_sUVDomain.GetMin();


          SmVector3d sSurfNormal, sCurveEvaluates[2];
          SER(pBS->EvaluateNormal(sDomainCorner, bUFromLeft, bVFromLeft,
                                  sSurfNormal));
      
          // -----------Explanation of CurveRelativeLocation-------------
          //
          //         --------2------>*    *: surface sample point
          //         ^               ^ 
          //         |               |
          //         3               1
          //         |               |
          //         |               |
          //         *----0---------->
          // 
          //-------------------------------------------------------------
    
          double dRelCurveLoc = (ii==0 || ii==3) ? 0.0 : 1.0;

          SER(pBSP->Evaluate(sParametricDomain.Evaluate(dRelCurveLoc),
                             1,                               // up to first derivative
                             (ii==0 || ii==3) ? FALSE : TRUE, // bFromLeft,
                             sCurveEvaluates));

          SmVector3d sCurveDirection( sCurveEvaluates[1] );
          SER(sCurveDirection.Unitize());

          sPlaneNormal = sCurveDirection * sSurfNormal;
          // sense (order of cross) is not important, it will be set anyway
        }

      // low work - degenerate boundary curve
      if (!bPlaneExists && !b3PointsCollinear)
          continue;
  
      // test: Is boundary curve control polygon on computed plane
      SmBoolean bIsPlanar;
      double dPlanarity;
      SER(pBSP->IsOnPlane(sPlaneBase, sPlaneNormal,
                          dTolerance,
                          bIsPlanar, 
                          dPlanarity));

      // low work - this curve not planar - go to next boundary curve
      if (!bIsPlanar)
          continue;
   
      if (b3PointsCollinear) 
        { 

          // What we have got now is 3 sample points of the boundary curve
          // being collinear, and the boundary curve being in the plane
          // composed of the average of the 3 points, and of the vector, 
          // perpendicular to both the surf normal and the curve direction 
          // in one of the 3 points. 
          // This plane is as good as any other plane this function
          // constructs. Moreover, it is fine for the most important
          // cases of planar, cylindric, etc surfaces, so just accept it. 
        }
    
      // test: does surface lie completely on one side of the bounding plane
      SmBoolean bIsSeparatingHalfspace = FALSE;
      SmBoolean bInvertNormal = FALSE;
      SER(sm_CheckBoundingPlaneSeparatingHalfspace
           (pBS, 
            sPlaneBase,
            sPlaneNormal,
            bIsSeparatingHalfspace,  // 1=yes, surf to one side of plane,0=no
            bInvertNormal));         // 1=plane normal points to surface,0=doesn't

      // low work - surface xsects bounding plane
      if (!bIsSeparatingHalfspace)
          continue;
  
      // set bounding plane normal to point away from surface
      if (bInvertNormal) 
          sPlaneNormal = -sPlaneNormal;

      // remember this boundary curve's state
      m_bBoundaryCurvePlanar[ii] = TRUE;
      m_vBasePoint[ii]           = sPlaneBase;
      m_vNormalVector[ii]        = sPlaneNormal;
      m_dPlaneTolerance[ii]      = dPlanarity;
      m_vSamplePoint[ii][0]      = sP1  ;
      m_vSamplePoint[ii][1]      = sP12 ;
      m_vSamplePoint[ii][2]      = sP2  ;
      m_vSamplePoint[ii][3]      = sP23 ;
      m_vSamplePoint[ii][4]      = sP3  ;
    } // end iter every boundary

  m_bPlanesComputed = TRUE;
  return SM_SUCCESS;

} // end SmPatchBoundaryBoundingPlanes::ComputeBoundingPlanes

SmPatchBoundaryBoundingPlanes::SmPatchBoundaryBoundingPlanes()
{}


/*******************************************************************//**
PURPOSE: Return the plane of the boundary curve if it exist.
            Plane is given by parameter space data.
            
NOTES: 
***********************************************************************/
SmStatus SmPatchBoundaryBoundingPlanes::GetBoundingPlane
  (SmSurfParamType eSurfParamArg,           // in : oneof SM_SP_U or SM_SP_V
   ULONG           nMinMaxArg,              // in : 0 for Min, 1 for Max
   SmBoolean     & rbPlaneExistsArg,        // out: TRUE=3d Projection of ith bounding isoParameter curve is planar
                                            //           and surface lies completely to one side of that plane.    
   SmBoolean     & rbBoundaryDegenerateArg, // out: TRUE=3d Projection of ith bounding isoParameter curve is degenerate
   SmPoint3d     & rsPlaneBaseArg,          // out: base point of bounding plane
   SmVector3d    & rsPlaneNormalArg,        // out: normal vec of bounding plane
   SmVector3d   *& pSamplePoint,            // out: Curve Sample Points to define Curve's bounding Box on bounding_plane
                                            //      sized:[SM_PB_SAMPLECOUNT]
   double        & rdToleranceArg)          // out: max deviation from plane
    
{
  ULONG nArrayIndex = sm_SurfparamToIndex(eSurfParamArg, nMinMaxArg);
  return GetNthBoundingPlane(nArrayIndex, 
                             rbPlaneExistsArg, 
                             rbBoundaryDegenerateArg, 
                             rsPlaneBaseArg, 
                             rsPlaneNormalArg,
                             pSamplePoint, 
                             rdToleranceArg);


} // end SmPatchBoundaryBoundingPlanes::GetBoundingPlane


/*******************************************************************//**
PURPOSE: Return the plane of the boundary curve if it exist.
            Plane is selected by an iterator index (0->3).
            
NOTES: 
***********************************************************************/
SmStatus SmPatchBoundaryBoundingPlanes::GetNthBoundingPlane
  (ULONG        nIndexArg,                 // in : An iterator index, [0->3]
   SmBoolean   & rbPlaneExistsArg,         // out: TRUE = ith boundary curve is planar and 
                                           //             Surface lies to one side of plane
   SmBoolean   & rbBoundaryDegenerateArg,  // out: TRUE = ith boundary curve is degenerate
   SmPoint3d   & rsPlaneBasePointArg,      // out: plane point, only set when rbPlaneExistsArg == TRUE
   SmVector3d  & rsPlaneNormalArg,         // out: plane normal, only set when rbPlaneExistsArg == TRUE
   SmVector3d *& pSamplePoint,             // out: pointer to array of boundary curve sample points.
                                           //      no new memory allocated, just referenced.
                                           //      sized:[SM_PB_SAMPLECOUNT]
   double      & rdToleranceArg)           // out: max distance from boundary curve control polygon to plane
{
  // init output
  rbBoundaryDegenerateArg = FALSE;

  if (!m_bPlanesComputed ) 
    { 
      // Does not seem to make sense crash in a boxtest-like function:
      SmStatus sStat = ComputeBoundingPlanes();
      if (sStat != SM_SUCCESS) 
        {
          rbPlaneExistsArg = FALSE;
          return SM_SUCCESS;
        }
    }

  if (!m_bBoundaryCurvePlanar[nIndexArg] ) 
    {
      rbPlaneExistsArg        = FALSE;
      rbBoundaryDegenerateArg = m_bBoundaryCurveDegenerate[nIndexArg];
      return SM_SUCCESS;
    }

  // set output
  rsPlaneBasePointArg = m_vBasePoint[nIndexArg];
  rsPlaneNormalArg    = m_vNormalVector[nIndexArg];
  pSamplePoint        = m_vSamplePoint[nIndexArg];
  rdToleranceArg      = m_dPlaneTolerance[nIndexArg];
  rbPlaneExistsArg    = TRUE;

  return SM_SUCCESS;

} // end SmPatchBoundaryBoundingPlanes::GetNthBoundingPlane


/********************************************************************
PURPOSE:  Compute a bezier approximation using derivative 
             information at corners of a patch.

NOTES:
*********************************************************************/
static SmStatus sm_ComputeBezierApproximation
  (const SmSurface  & crSurface,      // in :      
   const SmExtent2d & crUVDomain,     // in :      
   gw_SURFACE * pSurface)             // out: preallocated surface whose values get set
{
    SmVector3d sMat[2][2];
    gw_CPOINT **Pw  = pSurface->net->Pw;
    gw_KNOTVECTOR *pKNOTU = pSurface->knu;
    gw_KNOTVECTOR *pKNOTV = pSurface->knv;

    // Load knot vectors with domain bounds
    pKNOTU->m = 7;
    pKNOTV->m = 7;
    for (ULONG i=0; i<4; i++) {
        pKNOTU->U[i] = crUVDomain.GetMin().x;
        pKNOTV->U[i] = crUVDomain.GetMin().y;
        pKNOTU->U[i+4] = crUVDomain.GetMax().x;
        pKNOTV->U[i+4] = crUVDomain.GetMax().y;
    }

    SmPoint2d sSize = crUVDomain.GetSize();
    SmSurfParamType eSingDir;
    SmVector3d sDU, sDV, sDUV;

    // Now load control polygon at corners using hermite information.
    {
        SER(crSurface.Evaluate(crUVDomain.GetMin(),1,1,TRUE,TRUE,FALSE,sMat[0]));
        SmPoint3d sP00 = sMat[0][0];
        if (crSurface.IsSingularity(crUVDomain.GetMin(),eSingDir)) {
            SmPoint2d sUV = crUVDomain.GetMin();
            SmPoint2d sUVU(sUV.x+(sSize.x*SM_EFF_ZERO_SQRT),sUV.y);
            SmPoint2d sUVV(sUV.x,sUV.y+(sSize.y*SM_EFF_ZERO_SQRT));
            SER(crSurface.Evaluate(sUVU,1,1,TRUE,TRUE,FALSE,sMat[0]));
            sDU = sMat[1][0]*sSize.x;
            sDUV = sMat[1][1]*sSize.x*sSize.y;
            SER(crSurface.Evaluate(sUVV,1,1,TRUE,TRUE,FALSE,sMat[0]));
            sDV = sMat[0][1]*sSize.y;
            sDUV = (sDUV + sMat[1][1]*sSize.x*sSize.y)/2.0;
        }
        else {
            sDU = sMat[1][0]*sSize.x;
            sDV = sMat[0][1]*sSize.y;
            sDUV = sMat[1][1]*sSize.x*sSize.y;
        }
        COPY_XYZ(sP00,Pw[0][0]);
        SmPoint3d sP10 = sP00 + sDU/3.0;
        COPY_XYZ(sP10,Pw[1][0]);
        SmPoint3d sP01 = sP00 + sDV/3.0;
        COPY_XYZ(sP01,Pw[0][1]);
        SmPoint3d sP11 = sP10 + sP01 - sP00 + sDUV/9.0;
        COPY_XYZ(sP11,Pw[1][1]);
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
        if (bDebugMe) {
            smgfx_Erase();
            smgfx_SetColor(1,0,0);
            sP00.Draw();
            sm_GraphicsLoop();
            smgfx_SetColor(0,0,1);
            sP10.Draw();
            sm_GraphicsLoop();
            smgfx_SetColor(0,1,1);
            sP01.Draw();
            sm_GraphicsLoop();
            smgfx_SetColor(0,0,0);
            sP11.Draw();
            sm_GraphicsLoop();
        }
#endif

    }

    {
        SER(crSurface.Evaluate(crUVDomain.GetMax(),1,1,TRUE,TRUE,FALSE,sMat[0]));
        SmPoint3d sP00 = sMat[0][0];
        if (crSurface.IsSingularity(crUVDomain.GetMax(),eSingDir)) {
            SmPoint2d sUV = crUVDomain.GetMax();
            SmPoint2d sUVU(sUV.x-(sSize.x*SM_EFF_ZERO_SQRT),sUV.y);
            SmPoint2d sUVV(sUV.x,sUV.y-(sSize.y*SM_EFF_ZERO_SQRT));
            SER(crSurface.Evaluate(sUVU,1,1,TRUE,TRUE,FALSE,sMat[0]));
            sDU = sMat[1][0]*sSize.x;
            sDUV = sMat[1][1]*sSize.x*sSize.y;
            SER(crSurface.Evaluate(sUVV,1,1,TRUE,TRUE,FALSE,sMat[0]));
            sDV = sMat[0][1]*sSize.y;
            sDUV = (sDUV + sMat[1][1]*sSize.x*sSize.y)/2.0;
        }
        else {
            sDU = sMat[1][0]*sSize.x;
            sDV = sMat[0][1]*sSize.y;
            sDUV = sMat[1][1]*sSize.x*sSize.y;
        }
        COPY_XYZ(sP00,Pw[3][3]);
        SmPoint3d sP10 = sP00 - sDU/3.0;
        COPY_XYZ(sP10,Pw[2][3]);
        SmPoint3d sP01 = sP00 - sDV/3.0;
        COPY_XYZ(sP01,Pw[3][2]);
        SmPoint3d sP11 = sP10 + sP01 - sP00 + sDUV/9.0;
        COPY_XYZ(sP11,Pw[2][2]);
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
        if (bDebugMe) {
            smgfx_Erase();
            smgfx_SetColor(1,0,0);
            sP00.Draw();
            sm_GraphicsLoop();
            smgfx_SetColor(0,0,1);
            sP10.Draw();
            sm_GraphicsLoop();
            smgfx_SetColor(0,1,1);
            sP01.Draw();
            sm_GraphicsLoop();
            smgfx_SetColor(0,0,0);
            sP11.Draw();
            sm_GraphicsLoop();
        }
#endif
    }

    {
        SER(crSurface.Evaluate(crUVDomain.Evaluate(1.0,0.0),1,1,TRUE,TRUE,FALSE,sMat[0]));
        SmPoint3d sP00 = sMat[0][0];
        if (crSurface.IsSingularity(crUVDomain.Evaluate(1.0,0.0),eSingDir)) {
            SmPoint2d sUV = crUVDomain.Evaluate(1.0,0.0);
            SmPoint2d sUVU(sUV.x-(sSize.x*SM_EFF_ZERO_SQRT),sUV.y);
            SmPoint2d sUVV(sUV.x,sUV.y+(sSize.y*SM_EFF_ZERO_SQRT));
            SER(crSurface.Evaluate(sUVU,1,1,TRUE,TRUE,FALSE,sMat[0]));
            sDU = sMat[1][0]*sSize.x;
            sDUV = sMat[1][1]*sSize.x*sSize.y;
            SER(crSurface.Evaluate(sUVV,1,1,TRUE,TRUE,FALSE,sMat[0]));
            sDV = sMat[0][1]*sSize.y;
            sDUV = (sDUV + sMat[1][1]*sSize.x*sSize.y)/2.0;
        }
        else {
            sDU = sMat[1][0]*sSize.x;
            sDV = sMat[0][1]*sSize.y;
            sDUV = sMat[1][1]*sSize.x*sSize.y;
        }
        COPY_XYZ(sP00,Pw[3][0]);
        SmPoint3d sP10 = sP00 - sDU/3.0;
        COPY_XYZ(sP10,Pw[2][0]);
        SmPoint3d sP01 = sP00 + sDV/3.0;
        COPY_XYZ(sP01,Pw[3][1]);
        SmPoint3d sP11 = sP10 + sP01 - sP00 - sDUV/9.0; 
        COPY_XYZ(sP11,Pw[2][1]);
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
        if (bDebugMe) {
            smgfx_Erase();
            smgfx_SetColor(1,0,0);
            sP00.Draw();
            sm_GraphicsLoop();
            smgfx_SetColor(0,0,1);
            sP10.Draw();
            sm_GraphicsLoop();
            smgfx_SetColor(0,1,1);
            sP01.Draw();
            sm_GraphicsLoop();
            smgfx_SetColor(0,0,0);
            sP11.Draw();
            sm_GraphicsLoop();
        }
#endif
    }

    {
        SER(crSurface.Evaluate(crUVDomain.Evaluate(0.0,1.0),1,1,TRUE,TRUE,FALSE,sMat[0]));
        SmPoint3d sP00 = sMat[0][0];
        if (crSurface.IsSingularity(crUVDomain.Evaluate(0.0,1.0),eSingDir)) {
            SmPoint2d sUV = crUVDomain.Evaluate(0.0,1.0);
            SmPoint2d sUVU(sUV.x+(sSize.x*SM_EFF_ZERO_SQRT),sUV.y);
            SmPoint2d sUVV(sUV.x,sUV.y-(sSize.y*SM_EFF_ZERO_SQRT));
            SER(crSurface.Evaluate(sUVU,1,1,TRUE,TRUE,FALSE,sMat[0]));
            sDU = sMat[1][0]*sSize.x;
            sDUV = sMat[1][1]*sSize.x*sSize.y;
            SER(crSurface.Evaluate(sUVV,1,1,TRUE,TRUE,FALSE,sMat[0]));
            sDV = sMat[0][1]*sSize.y;
            sDUV = (sDUV + sMat[1][1]*sSize.x*sSize.y)/2.0;
        }
        else {
            sDU = sMat[1][0]*sSize.x;
            sDV = sMat[0][1]*sSize.y;
            sDUV = sMat[1][1]*sSize.x*sSize.y;
        }
        COPY_XYZ(sP00,Pw[0][3]);
        SmPoint3d sP10 = sP00 + sDU/3.0;
        COPY_XYZ(sP10,Pw[1][3]);
        SmPoint3d sP01 = sP00 - sDV/3.0;
        COPY_XYZ(sP01,Pw[0][2]);
        SmPoint3d sP11 = sP10 + sP01 - sP00 - sDUV/9.0; 
        COPY_XYZ(sP11,Pw[1][2]);
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
        if (bDebugMe) {
            smgfx_Erase();
            smgfx_SetColor(1,0,0);
            sP00.Draw();
            sm_GraphicsLoop();
            smgfx_SetColor(0,0,1);
            sP10.Draw();
            sm_GraphicsLoop();
            smgfx_SetColor(0,1,1);
            sP01.Draw();
            sm_GraphicsLoop();
            smgfx_SetColor(0,0,0);
            sP11.Draw();
            sm_GraphicsLoop();
        }
#endif
    }

    for (ULONG j=0; j<4; j++) {
        for (ULONG k=0; k<4; k++) {
            Pw[j][k].w = NL_NOW;
        }
    }

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
    if (bDebugMe) {
        sm_GraphicsLoop();
        smgfx_SetColor(1,0,0);
        SmBSplineSurface sBSS(pSurface,TRUE,crSurface.GetContext()) ;
        sBSS.SetContext(crSurface.GetContext());
        sBSS.Dump();
        sBSS.DrawNet();
        sBSS.DrawUV(5,5);
        sm_GraphicsLoop();
    }
#endif

    return SM_SUCCESS;

} // end sm_ComputeBezierApproximation

/*******************************************************************//**
PURPOSE: Compute Size Tolerance Values for Corners.

NOTES: Can be used when patch is flat enough.
***********************************************************************/
SmStatus SmSurfaceCache::ComputeSizeTolerances
 (const SmPoint3d & crU0V0Point,        // in : 3d corner of Surface(u0, v0) of target quad
  const SmPoint3d & crU1V0Point,        // in : 3d corner of Surface(u1, v0) of target quad
  const SmPoint3d & crU1V1Point,        // in : 3d corner of Surface(u1, v1) of target quad
  const SmPoint3d & crU0V1Point,        // in : 3d corner of Surface(u0, v1) of target quad
  SmVector2d      & rAspectRatio,       // out: [dMaxU/dMaxV, dMaxV/dMaxU] in 3d, except Max < dScaledZero, then 0.5
  SmVector2d      & rMaxSideLength3D,   // out: [dMaxU, dMaxV] in 3d,
  SmVector2d      & rMinSideLength3D)   // out: [dMinU, dMinV] in 3d, except Min < dScaledZero, then Max
{
  double dBotDist   = crU0V0Point.DistanceBetween(crU1V0Point);
  double dTopDist   = crU0V1Point.DistanceBetween(crU1V1Point);
  double dLeftDist  = crU0V0Point.DistanceBetween(crU0V1Point);
  double dRightDist = crU1V0Point.DistanceBetween(crU1V1Point);

  double dMaxU      = smos_Max(dBotDist,dTopDist);
  double dMaxV      = smos_Max(dLeftDist,dRightDist);

  double dMinU      = smos_Min(dBotDist,dTopDist);
  double dMinV      = smos_Min(dLeftDist,dRightDist);
  
  // Scaled Zero
  double dScaledZero = SM_EFF_ZERO * (1.0 + crU0V0Point.GetMaxDimension() + crU1V1Point.GetMaxDimension());

  // snap MinU and MinV values to MaxV when less than dTol, gwc???: why?
  if (dMinU < dScaledZero) { dMinU = dMaxU ; }
  if (dMinV < dScaledZero) { dMinV = dMaxV ; }

  // Set output
  rAspectRatio.x = dMaxV > dScaledZero ? dMaxU/dMaxV : 0.5; // gwc???: why snap to 0.5 when MaxV < dScaledZero?
  rAspectRatio.y = dMaxU > dScaledZero ? dMaxV/dMaxU : 0.5; // gwc???: why snap to 0.5 when MaxV < dScaledZero?

  rMaxSideLength3D.x = dMaxU;
  rMaxSideLength3D.y = dMaxV;

  rMinSideLength3D.x = dMinU;
  rMinSideLength3D.y = dMinV;

  return SM_SUCCESS;

} // end SmSurfaceCache::ComputeSizeTolerances (corner point version)

/*******************************************************************//**
PURPOSE: Compute Size Tolerance Values directly from surface patch.

NOTES: Used when patch is not flat enough.
***********************************************************************/
SmStatus SmSurfaceCache::ComputeSizeTolerances
 (const SmSurface  * pSurf,             // in : Target Surface
  const SmExtent2d & rDomain,           // in : Target Surface sub UVDomain
  SmVector2d       & rAspectRatio,      // out: [dMaxU/dMaxV, dMaxV/dMaxU] in 3d, except Max < dScaledZero, then 0.5
  SmVector2d       & rMaxSideLength3D,  // out: [dMaxU, dMaxV] in 3d,
  SmVector2d       & rMinSideLength3D)  // out: [dMinU, dMinV] in 3d, except Min < dScaledZero, then Max
{
  // Locals
  SmBSplineCurve * pBotUIsoCurve  = NULL, * pMidUIsoCurve  = NULL, * pTopUIsoCurve  = NULL ;
  SmBSplineCurve * pBotVIsoCurve  = NULL, * pMidVIsoCurve  = NULL, * pTopVIsoCurve  = NULL ;
  SmExtent1d       sUDomain     = rDomain.GetUInterval();
  SmExtent1d       sVDomain     = rDomain.GetVInterval();

  // Sample at low, high, and mid in each direction.
  // (On a big patch, low and high could both be degnerate.)

  // UVPoints
  SmPoint2d sUVBot = rDomain.Evaluate( 0.0, 0.0 );
  SmPoint2d sUVMid = rDomain.Evaluate( 0.5, 0.5 );
  SmPoint2d sUVTop = rDomain.Evaluate( 1.0, 1.0 );

  // IsoParamCurves
  SER( pSurf->CreateIsoParametricCurve(*GetContext(), SM_SP_U, sUVBot.x, 0.0, pBotUIsoCurve, &rDomain ));
  SER( pSurf->CreateIsoParametricCurve(*GetContext(), SM_SP_V, sUVBot.y, 0.0, pBotVIsoCurve, &rDomain ));

  SER( pSurf->CreateIsoParametricCurve(*GetContext(), SM_SP_U, sUVMid.x, 0.0, pMidUIsoCurve, &rDomain ));
  SER( pSurf->CreateIsoParametricCurve(*GetContext(), SM_SP_V, sUVMid.y, 0.0, pMidVIsoCurve, &rDomain ));

  SER( pSurf->CreateIsoParametricCurve(*GetContext(), SM_SP_U, sUVTop.x, 0.0, pTopUIsoCurve, &rDomain ));
  SER( pSurf->CreateIsoParametricCurve(*GetContext(), SM_SP_V, sUVTop.y, 0.0, pTopVIsoCurve, &rDomain ));

  // memory management
  SmObjDelete sClean1(pBotUIsoCurve);
  SmObjDelete sClean2(pBotVIsoCurve);
  SmObjDelete sClean3(pMidUIsoCurve);
  SmObjDelete sClean4(pMidVIsoCurve);
  SmObjDelete sClean5(pTopUIsoCurve);
  SmObjDelete sClean6(pTopVIsoCurve);

  // 3D Udir and Vdir IsoParamCurve lengths 
  double dBotVSize = pBotUIsoCurve->ApproximateLength( sVDomain, 8 );
  double dBotUSize = pBotVIsoCurve->ApproximateLength( sUDomain, 8 );

  double dMidVSize = pMidUIsoCurve->ApproximateLength( sVDomain, 8 );
  double dMidUSize = pMidVIsoCurve->ApproximateLength( sUDomain, 8 );

  double dVTopSize = pTopUIsoCurve->ApproximateLength( sVDomain, 8 );
  double dUTopSize = pTopVIsoCurve->ApproximateLength( sUDomain, 8 );

  // package lengths into SmPoint2d objs
  SmPoint2d sSizeLow(dBotUSize, dBotVSize) ;
  SmPoint2d sSizeMid(dMidUSize, dMidVSize) ;
  SmPoint2d sSizeTop(dUTopSize, dVTopSize) ;

  // Max IsoParamCurve lengths
  double dMaxU = smos_3Max( sSizeLow.x, sSizeMid.x, sSizeTop.x );
  double dMaxV = smos_3Max( sSizeLow.y, sSizeMid.y, sSizeTop.y );

  // Min IsoParamCurve lengths
  double dMinU = smos_3Min( sSizeLow.x, sSizeMid.x, sSizeTop.x );
  double dMinV = smos_3Min( sSizeLow.y, sSizeMid.y, sSizeTop.y );

  // Scaled Zero
  double dScaledZero = SM_EFF_ZERO * (1.0 + dMaxU + dMaxV );

  // snap MinU and MinV values to MaxV when less than dTol, gwc???: why?
  if (dMinU < dScaledZero) { dMinU = dMaxU; }
  if (dMinV < dScaledZero) { dMinV = dMaxV; }

  // set output
  rAspectRatio.x = dMaxV > dScaledZero ? dMaxU/dMaxV : 0.5; // gwc???: why snap to 0.5 when MaxV < dScaledZero?
  rAspectRatio.y = dMaxU > dScaledZero ? dMaxV/dMaxU : 0.5; // gwc???: why snap to 0.5 when MaxV < dScaledZero?

  rMaxSideLength3D.x = dMaxU;
  rMaxSideLength3D.y = dMaxV;

  rMinSideLength3D.x = dMinU;
  rMinSideLength3D.y = dMinV;

  return SM_SUCCESS;

} // end SmSurfaceCache::ComputeSizeTolerances (surface version)

/*******************************************************************//**
PURPOSE: Get information about corners of the Bezier patch.

NOTES: This is a very fast way to get this information.
***********************************************************************/
SmStatus SmSurfaceCache::GetCorners
  (gw_SURFACE       * pSurface,       // in : optional Surface to check
   const SmExtent2d & crPatchDomain,  // in : surface subDomain 
                                      //      expected to be the natural domain when pSurface != NULL
   SmPoint3d        & rU0V0Point,     // out: Surface(u0, v0) corner 3d point
   SmPoint3d        & rU1V0Point,     // out: Surface(u1, v0) corner 3d point
   SmPoint3d        & rU1V1Point,     // out: Surface(u1, v1) corner 3d point
   SmPoint3d        & rU0V1Point,     // out: Surface(u0, v1) corner 3d point
   SmExtent2d       & rDomain)        // out: surface domain for 4 corner points
{
  if (pSurface) 
    {
      gw_CPOINT    ** Pw     = pSurface->net->Pw ; NER(Pw) ;
      gw_KNOTVECTOR * pKNOTU = pSurface->knu ;     NER(pKNOTU) ;
      gw_KNOTVECTOR * pKNOTV = pSurface->knv ;     NER(pKNOTV) ;
      
      // set output
      rDomain.SetMinMax(pKNOTU->U[0],         
                        pKNOTV->U[0],        
                        pKNOTU->U[pKNOTU->m], 
                        pKNOTV->U[pKNOTV->m]) ;

      TO_EUCLID(Pw[0][0],                               rU0V0Point) ;
      TO_EUCLID(Pw[pSurface->net->n][0],                rU1V0Point) ;
      TO_EUCLID(Pw[pSurface->net->n][pSurface->net->m], rU1V1Point) ;
      TO_EUCLID(Pw[0][pSurface->net->m],                rU0V1Point) ; 
    }
  else // no gw_SURFACE available
    {
      // set output
      rDomain = crPatchDomain ;

      SER(m_cpSurface->EvaluatePoint(crPatchDomain.Evaluate(0.0,0.0), rU0V0Point)) ;
      SER(m_cpSurface->EvaluatePoint(crPatchDomain.Evaluate(1.0,0.0), rU1V0Point)) ;
      SER(m_cpSurface->EvaluatePoint(crPatchDomain.Evaluate(0.0,1.0), rU0V1Point)) ;
      SER(m_cpSurface->EvaluatePoint(crPatchDomain.Evaluate(1.0,1.0), rU1V1Point)) ;
    }
  return SM_SUCCESS;

} // end SmSurfaceCache::GetCorners

/*******************************************************************//**
PURPOSE: Constructor which initializes a alternate surface cache 
    object with a set of curves and some user data used to generate
    the curves.

NOTES: 
***********************************************************************/
SmAltSrfCache::SmAltSrfCache
  (const SmTArray<SmCurve*> * cp3DCurvesToCache,
   const SmTArray<SmCurve*> * cpUVCurvesToCache,
   SmAltSrfCacheType eSrfCacheType,
   double dApproxTol,
   double dAngleTol,
   const SmVector3d * cpUserVec,
   const SmBoolean  * cpUserBool,
   const long       * cpUserLong,
   const double     * cpUserDouble,
   const void       * cpUserPointer)
 : m_eSrfCacheType(eSrfCacheType), 
   m_dApproxTol(dApproxTol), 
   m_dAngleTol(dAngleTol), 
   m_vUserVec(0,0,0),
   m_bUserBool(FALSE), 
   m_lUserLong(0), 
   m_dUserDouble(0.0),
   m_cpUserPointer(cpUserPointer), 
   m_pCached3DCurves(NULL), 
   m_pCachedUVCurves(NULL)
{
    if (cp3DCurvesToCache) {
        m_pCached3DCurves = new (*GetContext()) SmTArray<SmCurve*>(*GetContext());
        m_pCached3DCurves->Append(*cp3DCurvesToCache);
    }
    if (cpUVCurvesToCache) {
        m_pCachedUVCurves = new (*GetContext()) SmTArray<SmCurve*>(*GetContext());
        m_pCachedUVCurves->Append(*cpUVCurvesToCache);
    }
    if (cpUserVec) m_vUserVec = *cpUserVec;
    if (cpUserBool) m_bUserBool = *cpUserBool;
    if (cpUserLong) m_lUserLong = *cpUserLong;
    if (cpUserDouble) m_dUserDouble = *cpUserDouble;

} // end SmAltSrfCache::SmAltSrfCache constructor

/*******************************************************************//**
PURPOSE: Destructor for the alternate surface cache.  It deletes the
    curves in the arrays and also deletes the arrays.

NOTES: 
***********************************************************************/
SmAltSrfCache::~SmAltSrfCache()
{
    if (m_pCached3DCurves) {
        SmObjsDelete<SmCurve*> sCleanUp(m_pCached3DCurves);
    }
    if (m_pCachedUVCurves) {
        SmObjsDelete<SmCurve*> sCleanUp(m_pCachedUVCurves);
    }
    if (m_pCached3DCurves) {
        delete m_pCached3DCurves; m_pCached3DCurves = NULL ;
    }
    if (m_pCachedUVCurves) {
        delete m_pCachedUVCurves; m_pCachedUVCurves = NULL ;
    }

} // end SmAltSrfCache::~SmAltSrfCache destructor

/*******************************************************************/ /**
 PURPOSE: Retrieve polar box for a BezierAux2d node.

 NOTES: If a polar box is unitilialized, it invokes BuildPolarBox which
 constructs a sequence of boxes from either the root or the first ancestor
 for which a polar box is defined.
 ***********************************************************************/
SmPolarBox& SmBezierAux2d::GetPolarBox()
{
    if ((this->m_sPolarBox).GetPolarBoxState() == SM_PS_UNINITIALIZED)
    {
        SE(this->BuildPolarBox());
    }
    return this->m_sPolarBox;
}

/*******************************************************************/ /**
 PURPOSE: Builds a sequence of polar boxes for SmBezierAux2d nodes, starting with
 either the root of the tree or the first anscestor for which a polar box is initialized
 and terminating in this node.

 NOTES: Was implemented to allow for lazy construction of polar boxes and thus
 faster tree construction for the tessellator, which does not use the polar boxes.
 ***********************************************************************/
SmStatus SmBezierAux2d::BuildPolarBox()
{
    // Arrays to collect chain of nodes up the tree, terminating in either the root
    // or something which has a polar box already
    SmTArray<SmTreeNode*>           pNodes;
    SmTArray<SmBezierAux2d*>        pAuxs;
    //SmObjsDelete<SmTreeNode*>       sCleanNodes(&pNodes);
    //SmObjsDelete<SmBezierAux2d*>    sCleanAuxs(&pAuxs);
    
    // Aux and node for where we are starting
    SmBezierAux2d* pCurrentAux      = this;
    SmTreeNode* pCurrentTreeNode    = pCurrentAux->m_pOwningTreeNode;

    // add these to the arrays
    pAuxs.Add(pCurrentAux);
    pNodes.Add(pCurrentTreeNode);

    // the parent and (potentially) the aux above the starting nodes
    SmTreeNode* pParent         = pCurrentTreeNode->m_pParent;
    SmBezierAux2d* pAuxParent   = NULL;

    // flags for stopping the hunt
    SmBoolean bFoundPolar   = FALSE;
    SmBoolean bFoundRoot    = FALSE;

    // No parent? then we starting at the root.

    if (!pParent)
    {
        bFoundRoot = TRUE;
    }
    // otherwise we have a generation above us, put that on the stack
    // along with the corresponding aux data
    else
    {
        pAuxParent = (SmBezierAux2d*)pParent->m_pData;
        pAuxs.Add(pAuxParent);
        pNodes.Add(pParent);

        // if this generation above us has a box, no need to keep looking
        if (pAuxParent->m_sPolarBox.GetPolarBoxState() != SM_PS_UNINITIALIZED)
        {
            bFoundPolar = TRUE;
        }
    }

    // while we are still looking for the root or a node with a box...
    while ((!bFoundPolar) && (!bFoundRoot))
    {

        // go one generation higher
        pParent = pParent->m_pParent;

        // if it doesn't exist, then the last thing we added to the list 
        // was the root, so exit.
        if (!pParent)
        {
            bFoundRoot = TRUE;
            continue;
        }

        // otherwise get aux data for the new parent
        pAuxParent = (SmBezierAux2d*)pParent->m_pData;

        // does that have a box? if so this is the last iteration.
        if (pAuxParent->m_sPolarBox.GetPolarBoxState() != SM_PS_UNINITIALIZED)
        {
            bFoundPolar = TRUE;
        }

        // add the parent and aux to array
        pAuxs.Add(pAuxParent);
        pNodes.Add(pParent);

    }

    // now we work backwards through the list. Have to
    // treat finding the root differently, the rest of the
    // list can be processed the same way.

    // locals
    SmTreeNode      * pLastNode = NULL;
    SmBezierAux2d   * pLastAux = NULL;
    pNodes.Pop(pLastNode);
    pAuxs.Pop(pLastAux);

    // holder for previous polar box
    SmPolarBox  * pLastPolar = NULL;

    // if we exited because we found a polar box, that is the 'top' box
    if (bFoundPolar)
    {
        pLastPolar = &(pLastAux->m_sPolarBox);
    }
    // otherwise we hit the root (can't have a box, else we would have stopped when we saw the box)
    else
    {
        const SmSurface* pLastBSS = pLastAux->mBA_pSurface;
        pLastBSS->CalculateBoundingBox(pLastAux->m_sUVDomain, NULL, NULL, &(pLastAux->m_sPolarBox), NULL, NULL);
        pLastPolar = &(pLastAux->m_sPolarBox);
    }

    // now we know we have a box in pLastBox, we build boxes toward the start of the array (our starting node)
    while (pAuxs.GetSize() > 0)
    {
        SmBezierAux2d* pAux = NULL;
        pAuxs.Pop(pAux);

        const SmSurface * pBSS = pAux->mBA_pSurface;

        // this addresses a bunch of assert messages by not using a bad initial guess
        if (pLastPolar -> GetPolarBoxState() == SM_PS_UNBOUNDED)
        {
            pLastPolar = NULL;
        }

        // now build the polar box and set us up for the next iteration.
        pBSS->CalculateBoundingBox(pAux->m_sUVDomain, NULL, NULL, &(pAux->m_sPolarBox), NULL, pLastPolar);
        pLastPolar = &(pAux->m_sPolarBox);
    }

    return SM_SUCCESS;
}
/*******************************************************************//**
PURPOSE: Determine if the cache matches the user data.  If it does then
    the curves can be used to satisfy this operation.

NOTES: 
***********************************************************************/
SmBoolean SmAltSrfCache::MatchCache
  (SmAltSrfCacheType  eSrfCacheType,   // in : cache parameter to compare
   double             dApproxTol,      // in : cache parameter to compare
   double             dAngleTol,       // in : cache parameter to compare
   const SmVector3d * cpUserVec,       // in : cache parameter to compare
   const SmBoolean  * cpUserBool,      // in : cache parameter to compare
   const long       * cpUserLong,      // in : cache parameter to compare
   const double     * cpUserDouble,    // in : cache parameter to compare
   const void       * cpUserPointer)   // in : cache parameter to compare
 const
{
  double dScale =  (cpUserVec)
                  ? cpUserVec->GetMaxDimension() * SM_EFF_ZERO
                  : SM_EFF_ZERO ;

  // when any AltSrfCache parameters differ - return FALSE
  if(   (eSrfCacheType != m_eSrfCacheType)
     || (cpUserBool    && *cpUserBool   != m_bUserBool)
     || (cpUserLong    && *cpUserLong   != m_lUserLong)
     || (cpUserPointer && cpUserPointer != m_cpUserPointer)
     || (cpUserVec     && cpUserVec->DistanceBetweenSquared(m_vUserVec) > dScale*dScale)
     || (cpUserDouble  && smos_Fabs(*cpUserDouble-m_dUserDouble) > SM_EFF_ZERO)
     || (smos_Fabs(dApproxTol-m_dApproxTol) > SM_EFF_ZERO)
     || (smos_Fabs(dAngleTol -m_dAngleTol)  > SM_EFF_ZERO)) return FALSE ;

  // else
  return TRUE ;

} // end SmAltSrfCache::MatchCache


/*******************************************************************//**
PURPOSE: Get a copy of the cached curves from the alternate surface
    cache.

NOTES: 
  Currently all of the curves must be SmBSplineCurves.

  The output curves are all copies of the curves stored in the SmAltSrfCache.
***********************************************************************/
SmStatus SmAltSrfCache::GetCopyOfCurves
  (const SmContext & crContext,                  // in : context for new object construction
   SmTArray<SmCurve*> * pCopyOf3DCurvesInCache,  // out: Optional 3D Curve copy output array,
                                                 //      NULL to skip curve copies.
   SmTArray<SmCurve*> * pCopyOfUVCurvesInCache)  // out: Optional UVCurve copy output array,
                                                 //      NULL to skip curve copies.
  const
{
  if (pCopyOf3DCurvesInCache) 
    {
      // init output
      NER(m_pCached3DCurves);
      pCopyOf3DCurvesInCache->ReSet();

      // load pointers to 3D curves
      pCopyOf3DCurvesInCache->Append(*m_pCached3DCurves);

      // for every Curve3d
      ULONG ii, lNumCopies = pCopyOf3DCurvesInCache->GetSize();
      for ( ii=0; ii<lNumCopies; ii++) 
        {
          // Replace cached curve with a new copy of the cached curve.
          SmBSplineCurve *pCachedBSC = SM_CAST_PTR(SmBSplineCurve,(*pCopyOf3DCurvesInCache)[ii]);
          NER(pCachedBSC);
          (*pCopyOf3DCurvesInCache)[ii] = new (crContext) SmBSplineCurve(*pCachedBSC);
          NER((*pCopyOf3DCurvesInCache)[ii]);
        } // end iter every m_pCached3DCurves member
    } // end get 3D Curve copies check

  if (pCopyOfUVCurvesInCache) 
    {
      // init output
      NER(m_pCachedUVCurves);
      pCopyOfUVCurvesInCache->ReSet();

      // load pointers to UVTrimCurves
      pCopyOfUVCurvesInCache->Append(*m_pCachedUVCurves);

      // for every UVTrimCurve
      for (ULONG i=0; i<pCopyOfUVCurvesInCache->GetSize(); i++) 
        {
          // Replace cached curve with a new copy of the cached curve.
          SmBSplineCurve *pCachedBSC = SM_CAST_PTR(SmBSplineCurve,(*pCopyOfUVCurvesInCache)[i]);
          NER(pCachedBSC);
          (*pCopyOfUVCurvesInCache)[i] = new (crContext) SmBSplineCurve(*pCachedBSC);
          NER((*pCopyOfUVCurvesInCache)[i]);

        } // end iter every UVTrimCurve
    } // end Get UVTrimCurve copies check

  return SM_SUCCESS;

} // end SmAltSrfCache::GetCopyOfCurves


/*******************************************************************//**
PURPOSE: Check to see if the cache has the curves being asked for
    in its alternate surface cache array.  If it does then load the
    output arrays and return TRUE.  If it does not then return FALSE.

NOTES: 
***********************************************************************/
SmBoolean SmSurfaceCache::HasCachedCurves
  (const SmContext & crContext,                   // in : context for constructor
   SmAltSrfCacheType eSrfCacheType,               // in : oneof 
                                                  //     SM_AS_SILHOUETTE         = contains silhouette curves trimmed
                                                  //                                to surface natural domain.    
                                                  //     SM_AS_TRIMMED_SILHOUETTE = contains silhouette curves trimmed
                                                  //                                to face boundaries.                  
   double             dApproxTol,                 // in : cache parameter to compare
   double             dAngleTol,                  // in : cache parameter to compare
   const SmVector3d * cpUserVec,                  // in : cache parameter to compare
   const SmBoolean  * cpUserBool,                 // in : cache parameter to compare
   const long       * cpUserLong,                 // in : cache parameter to compare
   const double     * cpUserDouble,               // in : cache parameter to compare
   const void       * cpUserPointer,              // in : cache parameter to compare
   SmTArray<SmCurve*> * pCopyOf3DCurvesInCache,   // out: Optional Copies of silhouette 3D curves,
                                                  //      NULL to skip curve copying 
   SmTArray<SmCurve*> * pCopyOfUVCurvesInCache)   // out: Optional Copies of silhouette UVTrimCurves,
                                                  //      NULL to skip curve copying
  const
{
  // for every AltCache
  for (ULONG i=0; i<m_lNumAltCaches; i++) 
    {
      // when cache parameters match input values
      if (m_apAltSrfCaches[i]->MatchCache(eSrfCacheType,
                                          dApproxTol, 
                                          dAngleTol,
                                          cpUserVec,
                                          cpUserBool,
                                          cpUserLong,
                                          cpUserDouble,
                                          cpUserPointer)) 
        {
          // get copies of curves and return
          m_apAltSrfCaches[i]->GetCopyOfCurves(crContext,pCopyOf3DCurvesInCache,
              pCopyOfUVCurvesInCache);

          return TRUE;

        } // end found a match check
    } // end iter every AltCach

  // didn't find a matching altCache
  return FALSE;

} // end SmSurfaceCache::HasCachedCurves

/*******************************************************************//**
PURPOSE: Add a set of curves of a given cache type with given data
    used to generate the curves to the curve cache.

NOTES: The curves sent in will be consumed (not copied) by the
    cache.
***********************************************************************/
SmStatus SmSurfaceCache::AddCurvesToCache
  (SmAltSrfCacheType eSrfCacheType,
   double dApproxTol,
   double dAngleTol,
   const SmVector3d * cpUserVec,
   const SmBoolean  * cpUserBool,
   const long       * cpUserLong,
   const double     * cpUserDouble,
   const void       * cpUserPointer,
   const SmTArray<SmCurve*> * cp3DCurvesToCache,
   const SmTArray<SmCurve*> * cpUVCurvesToCache)
{
    if (m_lNumAltCaches > SM_MAX_ALT_CACHES) { SER(SM_ERR); }

    if (m_lNumAltCaches == SM_MAX_ALT_CACHES) {
        m_lNumAltCaches = m_lNumAltCaches - 1;
        SM_ASSERT(m_apAltSrfCaches[m_lNumAltCaches] != NULL) ; delete m_apAltSrfCaches[m_lNumAltCaches] ; m_apAltSrfCaches[m_lNumAltCaches] = NULL ;
    }

    // Move caches up by one to make space at beginning.
    for (ULONG i=0; i<m_lNumAltCaches; i++) {
        ULONG idx = m_lNumAltCaches - i - 1;
        m_apAltSrfCaches[idx+1] = m_apAltSrfCaches[idx];
    }

    SmAltSrfCache *pNewASC = new (*GetContext()) SmAltSrfCache(
        cp3DCurvesToCache,cpUVCurvesToCache, eSrfCacheType,
        dApproxTol,dAngleTol,
        cpUserVec,cpUserBool,cpUserLong,cpUserDouble,cpUserPointer);
        
    m_apAltSrfCaches[0] = pNewASC;
    m_lNumAltCaches++;
    return SM_SUCCESS;

} // end SmSurfaceCache::AddCurvesToCache

/*******************************************************************//**
PURPOSE: Find the list of nodes in the trimmed surface tree whose
    UV domain intersects the input UVDomain

NOTES: returns leaf and interior nodes including nodes
    that just touch.
***********************************************************************/
SmStatus SmSurfaceCache::FindUVNodes
  (const SmExtent2d      & crUVDomain,     // in : test domain
   SmTArray<SmTreeNode*> & rNodes,         // out: all nodes that intersect the test domain
   SmBoolean               bCheckForPoles) // in : TRUE = expand search to include singular sides for poles
 const                                     //      default:[FALSE]
{
  // init output
  rNodes.ReSet();

  // locals
  SmSurfaceCache * pThis     = SM_CONST_CAST(SmSurfaceCache*,this);
  SmTree         * pTree     = pThis->GetTree();
  SmTreeNode     * pTop      = pTree->GetTopNode();
  SmTreeNode     * aData[100];
  SmTArray<SmTreeNode*> sStack(100,aData);

  // set up for poles
  ULONG      lPoles = 0 ;
  SmExtent1d sUIvl, sVIvl ;
  double     dUTol = 0.0, dVTol = 0.0 ;
  if(bCheckForPoles)
    {
      const SmSurface *cpSurface = GetSurface() ;
      SmExtent2d       sUVDomain = cpSurface->GetNaturalUVDomain() ;

      // save natural domain sizes
      sUIvl = sUVDomain.GetUInterval() ;
      sVIvl = sUVDomain.GetVInterval() ;
      dUTol = sUIvl.GetLength() / 100 ;
      dVTol = sVIvl.GetLength() / 100 ;

      // list crUVDomain boundaries near SrfNaturalDomain boundaries
      lPoles =   ((smos_Fabs(crUVDomain.GetMin().x-sUIvl.GetMin()) < dUTol) ? 1 : 0)
               + ((smos_Fabs(crUVDomain.GetMin().y-sVIvl.GetMin()) < dVTol) ? 2 : 0)
               + ((smos_Fabs(crUVDomain.GetMax().x-sUIvl.GetMax()) < dUTol) ? 4 : 0)
               + ((smos_Fabs(crUVDomain.GetMax().y-sVIvl.GetMax()) < dVTol) ? 8 : 0) ;

      // eliminate boundaries which are not poles
      if(lPoles)
        {
          ULONG lSrfPoles = cpSurface->GetSingularities() ;
          lPoles &= lSrfPoles ;
        }
   } // end set up for poles check

  // init a node stack
  sStack.Add(pTop);

  // while nodes remain to be processed
  while (sStack.GetSize() > 0) 
    {
      // get current node
      SmTreeNode    *pNode = (SmTreeNode*)sStack.GetLast();
      SmBezierAux2d *pAux  = (pNode) ? (SmBezierAux2d*)pNode->m_pData : NULL;
      sStack.RemoveLast();
      if ( pAux == NULL )
        {
          ERR_MSG(_T("Error: Surface cache: Null node"));
          continue;
        }
      SM_ASSERT(   pNode->m_eAuxDataType == SM_AD_AUX_DATA
                || pNode->m_eAuxDataType == SM_AD_BEZIER_SURFACE
                || pNode->m_eAuxDataType == SM_AD_BEZIER_SURFACE_HEAD);

      // keep nodes which are not disjoint
      SmBoolean bKeepForPosition = !pAux->m_sUVDomain.AreDisjoint(crUVDomain) ;

      // nodes near poles
      SmBoolean bKeepForPole = FALSE ;
      if(lPoles && !bKeepForPosition)
        { bKeepForPole =   ((lPoles & SM_SS_UMIN) && (smos_Fabs(pAux->m_sUVDomain.GetMin().x-sUIvl.GetMin()) < dUTol))
                        || ((lPoles & SM_SS_UMAX) && (smos_Fabs(pAux->m_sUVDomain.GetMax().x-sUIvl.GetMax()) < dUTol))
                        || ((lPoles & SM_SS_VMIN) && (smos_Fabs(pAux->m_sUVDomain.GetMin().y-sVIvl.GetMin()) < dVTol))
                        || ((lPoles & SM_SS_VMAX) && (smos_Fabs(pAux->m_sUVDomain.GetMax().y-sVIvl.GetMax()) < dVTol)) ;
        }

      // skip disjoint nodes not on a pole
      if(   !bKeepForPosition
         && !bKeepForPole) 
        { continue; }

      // arrive here when node domains intersect - add to output list
      rNodes.Add(pNode);
      
      // if node has children - add those to stack
      if (pNode->m_pChild1 != NULL) 
        { SM_ASSERT(pNode->m_pChild2 != NULL);

          // Interior node
          sStack.Add(pNode->m_pChild1);
          sStack.Add(pNode->m_pChild2);
        }

    } // end while searching for nodes with intersecting domains

  return SM_SUCCESS;

} // end SmSurfaceCache::FindUVNodes

/*******************************************************************//**
PURPOSE: Find an unmarked leaf node with the given classification.

NOTES: You need to pass in a stack which contains the current
    tree position you are searching.
***********************************************************************/
SmTreeNode * SmSurfaceCache::FindUnmarkedLeafNode
  (SmNodeClassType eNodeClassToFind,         // in : Type to search for
   SmTArray<SmTreeNode*> & rStack)           // i/o: Stack of Nodes to search
                                             //      Left in an unknown state
  const
{
  // while nodes are to be processed
  while (rStack.GetSize() > 0) 
    {
      // get stack top node
      SmTreeNode     *pNode = rStack.GetLast();
      SmBezierAux2d  *pAux  = (pNode) ? (SmBezierAux2d*)pNode->m_pData: NULL;
      rStack.RemoveLast();
      if ( pAux == NULL )
        {
          ERR_MSG(_T("Error: Surface cache: Null node"));
          continue;
        }

      SM_ASSERT(   pNode->m_eAuxDataType == SM_AD_AUX_DATA
                || pNode->m_eAuxDataType == SM_AD_BEZIER_SURFACE
                || pNode->m_eAuxDataType == SM_AD_BEZIER_SURFACE_HEAD);

      // skip already searched nodes
      if (pAux->m_bIsMarked) continue;

      // Add child Nodes to stack and continue.
      // Check leafNodes for containment
      if (pNode->m_pChild1 != NULL) 
        { 
          SM_ASSERT(pNode->m_pChild2 != NULL);
          rStack.Add(pNode->m_pChild1);
          rStack.Add(pNode->m_pChild2);
        }
      else // check leaf node for desired type
        { 
          if (pAux->m_eNodeClass == eNodeClassToFind) 
            {
              // return successes
              return pNode;
            }
        }
    } // end while searching for nodes

  return NULL;

} // end SmSurfaceCache::FindUnmarkedLeafNode


/*******************************************************************//**
PURPOSE: Get the lowest internal continuity of the surface within the
    given surface domain.  

NOTES: Returns SM_CT_CINFINITY when there are no internal knots
***********************************************************************/
SmContinuityType SmSurfaceCache::GetSurfaceContinuity
  (const SmExtent2d & crUVDomain)  // in : surface domain limit to test
{
  // init return
  SmContinuityType eRet = SM_CT_CINFINITY;

  // locals
  double           dData[100];
  SmTArray<double> dKnots(100,dData);
  SmExtent1d sUDomain = crUVDomain.GetUInterval() ;
  SmExtent1d sVDomain = crUVDomain.GetVInterval() ;

  // fetch U knots and cached continuity array
  SE(m_cpSurface->GetKnots(SM_SP_U,dKnots));
  const SmTArray<SmContinuityType> & rUConts = GetUContinuitiesArray();

  // remember lowest internal U continuity value
  if (rUConts.GetSize() > 0) 
    {
      // for every internal U knot
      for (ULONG iu=1; iu+1<rUConts.GetSize(); iu++)  // note: can't say rUConts.GetSize()-1
        {
          // remember lowest continuity seen in the given domain
          if(   rUConts[iu] <= eRet
             && sUDomain.ContainsValue(dKnots[iu])) 
            {
              eRet = rUConts[iu];
            }
        } // end iter every internal U knot value
    } // end UContinuitiesArray Size check 

  // fetch V knots and cached continuity array
  SE(m_cpSurface->GetKnots(SM_SP_V,dKnots));
  const SmTArray<SmContinuityType> & rVConts = GetVContinuitiesArray();

  // remember lowest internal V continuity value
  if (rVConts.GetSize() >0) 
    {
      // for every internal V knot
      for (ULONG iv=1; iv+1<rVConts.GetSize()-1; iv++)  // note: can't say rVConts.GetSize()-1
        {
          // remember lowest continuity seen in the given domain
          if(   rVConts[iv] <= eRet
             && sVDomain.ContainsValue(dKnots[iv])) 
            {
              eRet = rVConts[iv];
            }
        } // end iter every internal V knot value
    } // end VContinuitiesArray Size check

  // all done
  return eRet;

} // end SmSurfaceCache::GetSurfaceContinuity

/*******************************************************************//**
PURPOSE: Get all nodes connected to this node through shared common
            node boundaries that have the same m_eNodeClass values 
            and are not marked. 

NOTES:  Marks all nodes gathered into the region.
***********************************************************************/
void SmSurfaceCache::GetConnectedRegion
 (SmTreeNode *pStartLeafNode,                   // in : start node - must be a leaf node
  SmTArray<SmTreeNode *> &rConnectedNodes,      // out: connected region list including pStartLeafNode
  SmTreeNode *&rpClassifiedNeighborLeafNode)    // out: first neighbor found with a 
                                                //      m_eNodeClass == SM_NC_INSIDE or SM_NC_OUTSIDE classification
  const
{
  // init output
  rConnectedNodes.ReSet() ;
  rpClassifiedNeighborLeafNode = NULL ;

  // locals
  SmTreeNode  *aData256[256], *aData64[64] ;
  SmTArray<SmTreeNode*> sStack    (256,aData256);
  SmTArray<SmTreeNode*> sNeighbors(64, aData64);
  SmBezierAux2d        *pAux       = (SmBezierAux2d *)pStartLeafNode->m_pData ;
  SmNodeClassType       eNodeClass = pAux->m_eNodeClass ;

  // no work - Node is not from a surface spatial decomposition tree
  if(   pStartLeafNode->m_eAuxDataType !=  SM_AD_BEZIER_SURFACE                     
     && pStartLeafNode->m_eAuxDataType !=  SM_AD_BEZIER_SURFACE_HEAD 
     && pStartLeafNode->m_eAuxDataType !=  SM_AD_AUX_DATA)
    { return ; }

  // no work - pStartLeafNode is not a leaf node
  if(   pStartLeafNode->m_pChild1
     || pStartLeafNode->m_pChild2)
    { return ; }

  // start stack and output list with start node
  sStack.Add(pStartLeafNode) ;
  rConnectedNodes.Add(pStartLeafNode) ;

  // mark all nodes placed in the connectedNodes list
  pAux->m_bIsMarked = TRUE ;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ; 
      if(bDebugMe)
        { 
          SmTrimSrfCache *pTrmSrfCache = (SmTrimSrfCache *)this ; 
          SmTArray<SmTreeNode *> *pNodeList = NULL ;

          smgfx_Erase() ;
          smgfx_SetLook(1,2, 0,1,1) ; if(m_cpSurface) m_cpSurface->DrawUV() ; sm_GraphicsLoop() ; sm_GraphicsLoop() ;
          smgfx_SetLook(2,4, 1,0,1) ; pStartLeafNode->m_sBBox.Draw() ; sm_GraphicsLoop() ; sm_GraphicsLoop() ;
          for(ULONG jj=0;jj<3;jj++)
            {
              switch(jj) { case 0 : pNodeList = &rConnectedNodes ; smgfx_SetLook(1,6, 1,0,1) ; break ;
                           case 1 : pNodeList = &sStack ;          smgfx_SetLook(3,7, 1,1,0) ; break ;
                           case 2 : pNodeList = &sNeighbors ;      smgfx_SetLook(3,8, 0,1,1) ; break ;
                         }
              for(ULONG ii=0;ii<pNodeList->GetSize();ii++)
                {
                  SmTreeNode *pNode = (*pNodeList)[ii] ;
                  if (pNode->m_pChild1 != NULL) { continue; }

                  pNode->Draw() ; sm_GraphicsLoop() ; sm_GraphicsLoop() ;
                }
            }
          smgfx_SetLook(1,3, 1,0,0) ; pTrmSrfCache->Draw() ; sm_GraphicsLoop() ; sm_GraphicsLoop() ;
          sm_GraphicsLoop() ;
        }
#endif // SM_DEBUG_CODE

  // while stack continues to hold nodes
  while (sStack.GetSize() > 0) 
    {
      // get current node
      SmTreeNode     *pNode = (SmTreeNode*)sStack.GetLast();
      //SmBezierAux2d  *pAux  = (SmBezierAux2d*)pNode->m_pData;
      sStack.RemoveLast();
      SM_ASSERT(   pNode->m_eAuxDataType == SM_AD_AUX_DATA
                || pNode->m_eAuxDataType == SM_AD_BEZIER_SURFACE
                || pNode->m_eAuxDataType == SM_AD_BEZIER_SURFACE_HEAD);

      // get node's neighbors
      sNeighbors.ReSet() ;
      pNode->GetNeighbors(sNeighbors) ;

#ifdef SM_DEBUG_CODE
      if(bDebugMe)
        { 
          SmTrimSrfCache *pTrmSrfCache = (SmTrimSrfCache *)this ; 
          SmTArray<SmTreeNode *> *pNodeList = NULL ;

          smgfx_Erase() ;
          smgfx_SetLook(1,2, 0,1,1) ;   if(m_cpSurface) m_cpSurface->DrawUV() ; sm_GraphicsLoop() ; sm_GraphicsLoop() ;
          smgfx_SetLook(2,4, 1,.5,.5) ; pStartLeafNode->Draw(FALSE) ; sm_GraphicsLoop() ; sm_GraphicsLoop() ;
          smgfx_SetLook(3,5, .5,1,.5) ; pNode->Draw(FALSE) ; sm_GraphicsLoop() ; sm_GraphicsLoop() ;
          smgfx_SetLook(3,5, .5,.5,1) ; if(rpClassifiedNeighborLeafNode) rpClassifiedNeighborLeafNode->Draw(FALSE) ; sm_GraphicsLoop() ; sm_GraphicsLoop() ;
          
          for(ULONG jj=0;jj<3;jj++)
            {
              switch(jj) { case 0 : pNodeList = &rConnectedNodes ; smgfx_SetLook(1,6, 1,0,1) ; break ;
                           case 1 : pNodeList = &sStack ;          smgfx_SetLook(3,7, 1,1,0) ; break ;
                           case 2 : pNodeList = &sNeighbors ;      smgfx_SetLook(3,8, 0,1,1) ; break ;
                         }
              for(ULONG ii=0;ii<pNodeList->GetSize();ii++)
                {
                  SmTreeNode *pTreeNode = (*pNodeList)[ii] ;
                  if (pTreeNode->m_pChild1 != NULL) { continue; }

                  pTreeNode->Draw(FALSE) ; sm_GraphicsLoop() ; sm_GraphicsLoop() ;
                }
            }
          smgfx_SetLook(1,3, 1,0,0) ; pTrmSrfCache->Draw() ; sm_GraphicsLoop() ; sm_GraphicsLoop() ;
          sm_GraphicsLoop() ;
        }
#endif // SM_DEBUG_CODE

      // for every neighbor node
      for(ULONG ii=0;ii<sNeighbors.GetSize();ii++)
        {
          SmTreeNode      *pNeighborNode = sNeighbors[ii];
          SmBezierAux2d   *pNeighborAux  = (pNeighborNode) ? (SmBezierAux2d*)pNeighborNode->m_pData: NULL;
          if ( pNeighborAux == NULL )
            {
              ERR_MSG(_T("Error: Surface cache: Null node"));
              continue;
            }
      
          // skip marked nodes - save 1st non SM_NC_UNKNOWN pointer
          if(pNeighborAux->m_bIsMarked)
            {
              // save 1st non SM_NC_INSIDE/SM_NC_OUTSIDE pointer
              if(   rpClassifiedNeighborLeafNode == NULL
                 && (   pNeighborAux->m_eNodeClass == SM_NC_INSIDE
                     || pNeighborAux->m_eNodeClass == SM_NC_OUTSIDE))
                {
                  rpClassifiedNeighborLeafNode = pNeighborNode ;
                }
              continue ;
            }

          // for nodes of the same type as the start node
          if(pNeighborAux->m_eNodeClass == eNodeClass)
            {
              // add node to stack and connected list
              sStack.Add(pNeighborNode) ;
              rConnectedNodes.Add(pNeighborNode) ;

              // mark all nodes on the connected list
              pNeighborAux->m_bIsMarked = TRUE ; 
            }

        } // end iter neighbor nodes

    } // end while connected nodes are still on the stack

} // end SmTreeNode::GetConnectedRegion


/*******************************************************************//**
PURPOSE: Mark the containment of each node as inside/outside or on
    boundary of the face.

NOTES: 
  Assumes boundary nodes are marked as done in ImplantEdgeuse() and ImplantVertexuse(). 
  
    1. Nodes on boundaries are set as SM_NC_ON_BOUNDARY and marked.
    2. All ancestors of a boundary node are also set as SM_NC_ON_BOUNDARY.
    3. all parents with two marked children are also marked.

  This used to be based upon 2d raycasting but now uses loop containment.
  As such it can be called with m_bPointTestEnabled on or off.
***********************************************************************/
SmStatus SmSurfaceCache::MarkFaceContainment()
{
  // Propagate up ON_BOUNDARY flag and a mark indicating when
  // all subnodes have been classified.

  // locals
  SmTree     *pTree = GetTree();
  SmTreeNode *pTop = pTree->GetTopNode();
  SmTreeNode *aData[100];
  SmTArray<SmTreeNode*> sStack(100,aData);
  sStack.Add(pTop);

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
static ULONG lCount = 1 ; lCount++ ;
static ULONG lDebugCount = 0 ; 
#endif

// GWC:   slow method of propagation from leaf nodes replaced by
//        faster direct method added to ImplantEdgeuse() and ImplantVertexuse()
//        with new function PropagateToParents()
//
//        // propagate leafNode boundary classifications up to
//        // their parent nodes.
//        while (sStack.GetSize() > 0) 
//          {
//            // next node to process
//            SmTreeNode *pNode = sStack.GetLast();
//            SmBezierAux2d  *pAux  = (SmBezierAux2d *)pNode->m_pData;
//            sStack.RemoveLast();
//      
//            // check treenode state
//            SM_ASSERT(   pNode->m_eAuxDataType == SM_AD_AUX_DATA
//                      || pNode->m_eAuxDataType == SM_AD_BEZIER_SURFACE
//                      || pNode->m_eAuxDataType == SM_AD_BEZIER_SURFACE_HEAD);
//      
//            // if node is a parent, put its children on the stack
//            // else process the flags
//            if (pNode->m_pChild1 != NULL) 
//              {  
//                SM_ASSERT(pNode->m_pChild2 != NULL);
//      
//                // place children on stack
//                sStack.Add(pNode->m_pChild1);
//                sStack.Add(pNode->m_pChild2);
//              } // end interior node branch
//      
//            else // leaf node branch
//              { 
//                // if NodeClass is on the boundary
//                if (pAux->m_eNodeClass == SM_NC_ON_BOUNDARY) 
//                  {
//                    // mark the node as processed
//                    pAux->m_bIsMarked = TRUE;
//      
//                    // propagate the BOUNDARY classification to parents
//                    // And when both children are marked propagate
//                    //    also propagate the mark
//                    SmTreeNode *pParent = pNode->m_pParent;
//                    while (pParent != NULL) 
//                      {
//                        SmBezierAux2d *pParentAux = (SmBezierAux2d*)pParent->m_pData;
//                        SmBezierAux2d *pAux1 = (SmBezierAux2d*)pParent->m_pChild1->m_pData;
//                        SmBezierAux2d *pAux2 = (SmBezierAux2d*)pParent->m_pChild2->m_pData;
//                        pParentAux->m_eNodeClass = SM_NC_ON_BOUNDARY;
//                        if (   pAux1->m_bIsMarked
//                            && pAux2->m_bIsMarked) 
//                          {
//                            pParentAux->m_bIsMarked = TRUE;
//                          }
//                        pParent = pParent->m_pParent;
//                      }
//                  } // end boundary classification check
//              } // end leaf node branch
//          } // end While to propagate leafNode boundary classifications to ParentNodes
//      
//        // reset the stack
//        sStack.ReSet();
//        sStack.Add(pTop);

  // local array for neighboring Nodes
  SmTreeNode *aData2[256];
  SmTArray<SmTreeNode*> sConnectedNodes(256,aData2);

  // Set every LeafNode->Classification 
  //  1. from INSIDE/OUTSIDE classified NeighborNode if possible,
  //  2. else by point based rayFire. 
  //
  // With  luck we should only have to do one or two rayfires per
  // region being classified.  If we end up doing more then that, then we
  // can propagate to a second level.

  // find a 1st unMarked leafNode with classification == SM_NC_UNKNOWN
  SmTreeNode *pCurrNode = FindUnmarkedLeafNode(SM_NC_UNKNOWN, sStack);

#ifdef SM_DEBUG_CODE        
  if(bDebugMe)
    { 
      SmTrimSrfCache *pTrmSrfCache = (SmTrimSrfCache *)this ;

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,1,1) ; if(m_cpSurface) m_cpSurface->DrawUV() ; sm_GraphicsLoop() ; sm_GraphicsLoop() ;
      smgfx_SetLook(2,3, 1,0,1) ; pCurrNode->m_sBBox.Draw() ; sm_GraphicsLoop() ; sm_GraphicsLoop() ;
      smgfx_SetLook(2,3, 1,0,1) ; pTrmSrfCache->Draw() ; sm_GraphicsLoop() ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // while unMarked Nodes remain with classification SM_NC_UNKNOWN
  while (pCurrNode != NULL) 
    {
      SmBezierAux2d   *pAux                        = (SmBezierAux2d*)pCurrNode->m_pData;
      SmTreeNode      *pClassifiedNeighborLeafNode = NULL ;
      SmNodeClassType  eFoundNodeClass             = SM_NC_UNKNOWN ;
      SmObject        *pFoundFace                  = NULL;

      // get all nodes in connected region of SM_NC_UNKNOWN nodes
      // gathered nodes get marked i.e. (m_bIsMarked = TRUE ;)
      sConnectedNodes.ReSet() ;
      GetConnectedRegion(pCurrNode, sConnectedNodes, pClassifiedNeighborLeafNode) ; 

#ifdef SM_DEBUG_CODE
      if(bDebugMe || lDebugCount == lCount)
        { 
          SmTrimSrfCache *pTrmSrfCache = (SmTrimSrfCache *)this ; 

          smgfx_Erase() ;
          smgfx_SetLook(1,2, 0,1,1) ; if(m_cpSurface) m_cpSurface->DrawUV() ; sm_GraphicsLoop() ; sm_GraphicsLoop() ;
          smgfx_SetLook(3,6, 1,0,1) ; if(pCurrNode) pCurrNode->Draw(FALSE) ; sm_GraphicsLoop() ; sm_GraphicsLoop() ;
          smgfx_SetLook(2,5, 1,.5,.5) ; for(ULONG ii=0;ii<sConnectedNodes.GetSize();ii++)
                                          {
                                            if (sConnectedNodes[ii]->m_pChild1 != NULL) { continue; }
                                            sConnectedNodes[ii]->Draw(FALSE) ; sm_GraphicsLoop() ; sm_GraphicsLoop() ;
                                          }
          smgfx_SetLook(1,2, 1,0,0) ; pTrmSrfCache->Draw() ; sm_GraphicsLoop() ; sm_GraphicsLoop() ;
          sm_GraphicsLoop() ;
        }
#endif // SM_DEBUG_CODE


      // get classification from the classified neighbor leaf node if possible
      if(pClassifiedNeighborLeafNode)
        {
          SmBezierAux2d *pFoundAux = (SmBezierAux2d*)pClassifiedNeighborLeafNode->m_pData ;
          eFoundNodeClass = pFoundAux->m_eNodeClass ;
          pFoundFace      = pFoundAux->m_pFace ;
        }
                 
      // when no classified neighbor was found - cast a ray
      if (eFoundNodeClass == SM_NC_UNKNOWN) 
        {
          SmObject                  *pObjectInOrOn = NULL;
          SmPointClassificationType  eClassification;
          SmZoneTol3d sSrcZoneTol3d = SmTol::GetZoneTol3d(pAux->m_pFace) ;

          // classify this SurfaceCache Node. 
          // This used to be based on ray casting but now uses cheaper loop-containment
          SmPoint2d sTestPt = pAux->m_sUVDomain.Evaluate(0.5,0.5);
          SER(PointClassify(sTestPt,
                            sSrcZoneTol3d,
                            eClassification, 
                            pObjectInOrOn));

          // remember the Node is in or out of the Face
          if (eClassification == SM_PC_FACE) { eFoundNodeClass = SM_NC_INSIDE;
                                               // composite faces are not stored
                                               // only their child faces
                                               pFoundFace      = pObjectInOrOn;
                                             }
          else                               { eFoundNodeClass = SM_NC_OUTSIDE;
                                             }
        } // end need to cast ray check

      // propagate this node's classification to all its connected nodes
      for (ULONG j=0; j<sConnectedNodes.GetSize(); j++) 
        {
          SmTreeNode     *pConnectedNode = sConnectedNodes[j];
          SmBezierAux2d  *pConnectedAux  = (SmBezierAux2d*)pConnectedNode->m_pData;
          
          // internal nodes have been skipped
          SM_ASSERT(   pConnectedNode->m_pChild1 == NULL
                    && pConnectedNode->m_pChild2 == NULL) ;

          // All nodes placed on the connectedNodes list have been marked
          SM_ASSERT(   pConnectedAux->m_bIsMarked == TRUE) ;
                       
          // set the node classification and mark state
          pConnectedAux->m_eNodeClass = eFoundNodeClass;
          pConnectedAux->m_pFace      = pFoundFace;
          
          // propagate information up tree
          pConnectedNode->PropagateToParents() ;

        } // end iter every adjacent node

      // search for the next unmarked and unclassified node
      sStack.ReSet();
      sStack.Add(pTop);

      pCurrNode = FindUnmarkedLeafNode(SM_NC_UNKNOWN, sStack);

#ifdef SM_DEBUG_CODE
      if(bDebugMe || lDebugCount == lCount)
        { 
          SmTrimSrfCache *pTrmSrfCache = (SmTrimSrfCache *)this ; 
          SmFace         *pFace        = (SmFace *)m_cpSurface->GetFace() ; 

          smgfx_Erase() ;
          smgfx_SetLook(1,2, 0,1,1) ; if(m_cpSurface) m_cpSurface->DrawUV() ; sm_GraphicsLoop() ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 0,0,0) ; if(pFace) pFace->Draw(SM_DM_WIREFRAME) ; sm_GraphicsLoop() ; sm_GraphicsLoop() ;
          smgfx_SetLook(3,6, 1,0,1) ; if(pCurrNode) pCurrNode->Draw(FALSE) ; sm_GraphicsLoop() ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,3, 1,0,0) ; pTrmSrfCache->Draw() ; sm_GraphicsLoop() ; sm_GraphicsLoop() ;
          sm_GraphicsLoop() ;
        }
      lCount++ ;
#endif // SM_DEBUG_CODE

    } // end while unmarked and unClassified nodes remain to be classified

  // all done
  return SM_SUCCESS;

} // end SmSurfaceCache::MarkFaceContainment

//      /*******************************************************************//**
//      PURPOSE: Mark the containment of each node as inside/outside or on
//          boundary of the face.
//      
//      NOTES: Nodes on boundaries are already marked.
//      ***********************************************************************/
//      SmStatus SmSurfaceCache::MarkFaceContainment()
//      {
//        // Propagate up ON_BOUNDARY flag and a mark indicating when
//        // all subnodes have been classified.
//      
//        // locals
//        SmTree     *pTree = GetTree();
//        SmTreeNode *pTop = pTree->GetTopNode();
//        SmTreeNode *aData[100];
//        SmTArray<SmTreeNode*> sStack(100,aData);
//        sStack.Add(pTop);
//      
//      #ifdef SM_DEBUG_CODE
//      SmBoolean bDebugMe = FALSE ;
//      ULONG     lCount   = 0 ;
//      #endif
//      
//        // propagate leafNode boundary classifications up to
//        // their parent nodes.
//        while (sStack.GetSize() > 0) 
//          {
//            // next node to process
//            SmTreeNode *pNode = sStack.GetLast();
//            SmBezierAux2d  *pAux  = (SmBezierAux2d *)pNode->m_pData;
//            sStack.RemoveLast();
//      
//            // check treenode state
//            SM_ASSERT(   pNode->m_eAuxDataType == SM_AD_AUX_DATA
//                      || pNode->m_eAuxDataType == SM_AD_BEZIER_SURFACE
//                      || pNode->m_eAuxDataType == SM_AD_BEZIER_SURFACE_HEAD);
//      
//            // if node is a parent, put its children on the stack
//            // else process the flags
//            if (pNode->m_pChild1 != NULL) 
//              {  
//                // Interior node
//                SM_ASSERT(pNode->m_pChild2 != NULL);
//      
//                // place children on stack
//                sStack.Add(pNode->m_pChild1);
//                sStack.Add(pNode->m_pChild2);
//              } // end interior node branch
//      
//            else // leaf node branch
//              { 
//                // if NodeClass is on the boundary
//                if (pAux->m_eNodeClass == SM_NC_ON_BOUNDARY) 
//                  {
//                    // mark the node as processed
//                    pAux->m_bIsMarked = TRUE;
//      
//                    // propagate the BOUNDARY classification to parents
//                    // And when both children are marked propagate
//                    //    also propagate the mark
//                    SmTreeNode *pParent = pNode->m_pParent;
//                    while (pParent != NULL) 
//                      {
//                        SmBezierAux2d *pParentAux = (SmBezierAux2d*)pParent->m_pData;
//                        SmBezierAux2d *pAux1 = (SmBezierAux2d*)pParent->m_pChild1->m_pData;
//                        SmBezierAux2d *pAux2 = (SmBezierAux2d*)pParent->m_pChild2->m_pData;
//                        pParentAux->m_eNodeClass = SM_NC_ON_BOUNDARY;
//                        if (   pAux1->m_bIsMarked
//                            && pAux2->m_bIsMarked) 
//                          {
//                            pParentAux->m_bIsMarked = TRUE;
//                          }
//                        pParent = pParent->m_pParent;
//                      }
//                  } // end boundary classification check
//              } // end leaf node branch
//          } // end While to propagate leafNode boundary classifications to ParentNodes
//      
//        // reset the stack
//        sStack.ReSet();
//        sStack.Add(pTop);
//      
//        // local array for neighboring Nodes
//        SmTreeNode *aData2[100];
//        SmTArray<SmTreeNode*> sAdjacentNodes(100,aData2);
//      
//        // Set every LeafNode->Classification 
//        //  1. from INSIDE/OUTSIDE classified NeighborNode if possible,
//        //  2. else by point based rayFire. 
//        //
//        // With  luck we should only have to do one or two rayfires per
//        // region being classified.  If we end up doing more then that, then we
//        // can propagate to a second level.
//      
//        // find a 1st unMarked leafNode with classification == SM_NC_UNKNOWN
//        SmTreeNode *pCurrNode = FindUnmarkedLeafNode(SM_NC_UNKNOWN, sStack);
//      
//      #ifdef SM_DEBUG_CODE
//        if(bDebugMe)
//          { 
//            SmTrimSrfCache *pTrmSrfCache = (SmTrimSrfCache *)this ;
//            SmFace         *pFace        = (SmFace *)m_cpSurface->GetFace() ; 
//            smgfx_Erase() ;
//            smgfx_SetLook(1,2, 0,1,1) ; if(m_cpSurface) m_cpSurface->DrawUV() ; sm_GraphicsLoop() ; sm_GraphicsLoop() ;
//            smgfx_SetLook(1,2, 0,0,0) ; if(pFace) pFace->Draw(SM_DM_WIREFRAME) ; sm_GraphicsLoop() ; sm_GraphicsLoop() ;
//            smgfx_SetLook(2,4, 1,0,1) ; pCurrNode->m_sBBox.Draw() ; sm_GraphicsLoop() ; sm_GraphicsLoop() ;
//            sm_GraphicsLoop() ;
//          }
//      #endif // SM_DEBUG_CODE
//      
//        // while unMarked Nodes remain with classification SM_NC_UNKNOWN
//        while (pCurrNode != NULL) 
//          {
//            SmBezierAux2d *pAux = (SmBezierAux2d*)pCurrNode->m_pData;
//      
//            // find all nodes whose domain intersect thisNode's domain 
//            SER(FindUVNodes(pAux->m_sUVDomain,sAdjacentNodes));
//      
//      #ifdef SM_DEBUG_CODE
//            if(bDebugMe)
//              { 
//                SmTrimSrfCache *pTrmSrfCache = (SmTrimSrfCache *)this ; 
//                SmFace         *pFace        = (SmFace *)m_cpSurface->GetFace() ; 
//      
//                smgfx_Erase() ;
//                smgfx_SetLook(1,2, 0,1,1) ; if(m_cpSurface) m_cpSurface->DrawUV() ; sm_GraphicsLoop() ; sm_GraphicsLoop() ;
//                smgfx_SetLook(1,2, 0,0,0) ; if(pFace) pFace->Draw(SM_DM_WIREFRAME) ; sm_GraphicsLoop() ; sm_GraphicsLoop() ;
//                smgfx_SetLook(2,4, 1,0,1) ; pCurrNode->m_sBBox.Draw() ; sm_GraphicsLoop() ; sm_GraphicsLoop() ;
//                smgfx_SetLook(1,4, 0,0,0) ; for(ULONG ii=0;ii<sAdjacentNodes.GetSize();ii++)
//                                              {
//                                                if (sAdjacentNodes[ii]->m_pChild1 != NULL) { continue; }
//                                                SmPoint3d sMidPoint = sAdjacentNodes[ii]->m_sBBox.GetMid() ;
//                                                sMidPoint.Draw() ; sm_GraphicsLoop() ; sm_GraphicsLoop() ;
//                                              }
//                smgfx_SetLook(1,2, 1,0,0) ; pTrmSrfCache->Draw() ; sm_GraphicsLoop() ; sm_GraphicsLoop() ;
//                sm_GraphicsLoop() ;
//              }
//      #endif // SM_DEBUG_CODE
//      
//            // for every neighboring node - seek one that is classified INSIDE or OUTSIDE
//            SmNodeClassType eFoundNodeClass = SM_NC_UNKNOWN;
//            SmObject      * pFoundFace      = NULL;
//            for (ULONG i=0; i<sAdjacentNodes.GetSize(); i++) 
//              {
//                SmTreeNode *pAdjNode = sAdjacentNodes[i];
//      
//                // skip parent nodes
//                if (pAdjNode->m_pChild1 != NULL) 
//                  { continue; }
//      
//                // skip neighbors touching at a corner
//                SmBezierAux2d *pAdjAux = (SmBezierAux2d*)pAdjNode->m_pData;
//                if (pAdjAux->m_sUVDomain.IsTouchingOnePointAt2DCorner(pAux->m_sUVDomain)) 
//                  {
//                    //MSG(_T("TA!")); // Temp
//                    continue;
//                  }
//      
//                // when neighbor is classified as INSIDE or OUTSIDE
//                if(   pAdjAux->m_eNodeClass == SM_NC_INSIDE
//                   || pAdjAux->m_eNodeClass == SM_NC_OUTSIDE) 
//                  {
//                    // remember the hit
//                    eFoundNodeClass = pAdjAux->m_eNodeClass;
//                    if (pAdjAux->m_eNodeClass == SM_NC_INSIDE) 
//                      {
//                        pFoundFace = pAdjAux->m_pFace;
//                      }
//      
//                    // and stop the search
//                    break;
//                  }
//              } // end iter every neighboring node seeking a classified Node
//      
//            // when no classified neighbor was found
//            if (eFoundNodeClass == SM_NC_UNKNOWN) 
//              {
//                SmObject                  *pObjectInOrOn;
//                SmPointClassificationType  eClassification;
//                SmZoneTol3d sSrcZoneTol3d = SmTol::GetZoneTol3d(pAux->m_pFace) ;
//      
//                // classify this SurfaceCache Node by casting a 2d ray from its midPoint
//                SER(PointClassify(pAux->m_sUVDomain.Evaluate(0.5,0.5), sSrcZoneTol3d
//                                  eClassification, pObjectInOrOn));
//      
//                // remember the Node is in or out of the Face
//                if (eClassification == SM_PC_FACE) { eFoundNodeClass = SM_NC_INSIDE;
//                                                     // composite faces are not stored
//                                                     // only their child faces
//                                                     pFoundFace      = pObjectInOrOn;
//                                                   }
//                else                               { eFoundNodeClass = SM_NC_OUTSIDE;
//                                                   }
//              } // end need to cast ray check
//      
//            // propagate this node's classification to all its neighbor nodes
//            for (ULONG j=0; j<sAdjacentNodes.GetSize(); j++) 
//              {
//                SmTreeNode *pAdjNode = sAdjacentNodes[j];
//      
//                // skip internal nodes
//                if (pAdjNode->m_pChild1 != NULL) 
//                  { continue; }
//      
//                // for unMarked Nodes
//                SmBezierAux2d *pAdjAux = (SmBezierAux2d*)pAdjNode->m_pData;
//                if (!pAdjAux->m_bIsMarked) 
//                  {
//                    // skip nodes touching at a corner
//                    if (pAdjAux->m_sUVDomain.IsTouchingOnePointAt2DCorner(pAux->m_sUVDomain)) 
//                      {
//                        //MSG(_T("TA!")); // Temp
//                        continue;
//                      }
//      
//                    // set the node classification and mark state
//                    pAdjAux->m_bIsMarked  = TRUE;
//                    pAdjAux->m_eNodeClass = eFoundNodeClass;
//                    pAdjAux->m_pFace      = pFoundFace;
//      
//                    // Propagate information up tree
//                    SmTreeNode *pParent = pAdjNode->m_pParent;
//                    while (pParent != NULL) 
//                      {
//                        SmBezierAux2d *pParentAux = (SmBezierAux2d*)pParent->m_pData;
//                        SmBezierAux2d *pAux1      = (SmBezierAux2d*)pParent->m_pChild1->m_pData;
//                        SmBezierAux2d *pAux2      = (SmBezierAux2d*)pParent->m_pChild2->m_pData;
//      
//                        // when both children are not marked - quit moving up
//                        if (   0 == pAux1->m_bIsMarked
//                            && 0 == pAux2->m_bIsMarked) 
//                          {
//                            break ;
//                          }
//      
//                        // when both children are marked - mark the parent
//                        if (   pAux1->m_bIsMarked
//                            && pAux2->m_bIsMarked) 
//                          {
//                            pParentAux->m_bIsMarked = TRUE;
//                          }
//      
//                        // when both children have this node's classification type
//                        if (   pAux1->m_eNodeClass == eFoundNodeClass
//                            && pAux2->m_eNodeClass == eFoundNodeClass) 
//                          { // set the parent's classification
//                            pParentAux->m_eNodeClass = eFoundNodeClass;
//                            pParentAux->m_pFace      = pFoundFace;
//                          }
//      
//                        // continue with grandParents
//                        pParent = pParent->m_pParent;
//                      } // end while parents to check
//                  } // end found an unmarked node
//              } // end iter every adjacent node
//      
//            // search for the next unmarked and unclassified node
//            pCurrNode = FindUnmarkedLeafNode(SM_NC_UNKNOWN,sStack);
//      
//      #ifdef SM_DEBUG_CODE
//            if(bDebugMe)
//              { 
//                SmTrimSrfCache *pTrmSrfCache = (SmTrimSrfCache *)this ; 
//                SmFace         *pFace        = (SmFace *)m_cpSurface->GetFace() ; 
//      
//                smgfx_Erase() ;
//                smgfx_SetLook(1,2, 0,1,1) ; if(m_cpSurface) m_cpSurface->DrawUV() ; sm_GraphicsLoop() ; sm_GraphicsLoop() ;
//                smgfx_SetLook(1,2, 0,0,0) ; if(pFace) pFace->Draw(SM_DM_WIREFRAME) ; sm_GraphicsLoop() ; sm_GraphicsLoop() ;
//                smgfx_SetLook(2,4, 1,0,1) ; pCurrNode->m_sBBox.Draw() ; sm_GraphicsLoop() ; sm_GraphicsLoop() ;
//                smgfx_SetLook(1,4, 0,0,0) ; for(ULONG ii=0;ii<sAdjacentNodes.GetSize();ii++)
//                                              {
//                                                if (sAdjacentNodes[ii]->m_pChild1 != NULL) { continue; }
//                                                SmPoint3d sMidPoint = sAdjacentNodes[ii]->m_sBBox.GetMid() ;
//                                                sMidPoint.Draw() ; sm_GraphicsLoop() ; sm_GraphicsLoop() ;
//                                              }
//                smgfx_SetLook(1,2, 1,0,0) ; pTrmSrfCache->Draw() ; sm_GraphicsLoop() ; sm_GraphicsLoop() ;
//                sm_GraphicsLoop() ;
//              }
//            lCount++ ;
//      #endif // SM_DEBUG_CODE
//      
//          } // end while unmarked and unClassified nodes remain to be classified
//      
//        return SM_SUCCESS;
//      
//      } // end SmSurfaceCache::MarkFaceContainment

/*******************************************************************//**
PURPOSE: Subdivide the given node.

NOTES: Turn one node into a parent, child1, child2 triplet
   in which both children get about 1/2 of the parent's original geometry.

   child memory is allocated for
     a. SmTreeNode                                      from m_sNodeMgr
     b. (bOnlySubdivideAuxData == TRUE ) SmBezierAux2d  from m_sAuxMgr
        (bOnlySubdivideAuxData == FALSE) SmBezierPatch  from m_sBezMgr
***********************************************************************/
SmStatus SmSurfaceCache::SubdivideNode
  (SmTreeNode     *pNode,                  // in : target node      
   SmSurfParamType eSubdivideDirection,    // in : saved in pNode->m_pData->m_eSplitDir.
                                           //      oneof SM_SP_U: Child1 = left, Child2 = right
                                           //            SM_SP_V: Child1 = bot,  Child2 = top
   SmBoolean       bOnlySubdivideAuxData,  // in : TRUE=no Bezier patch shape to split
                                           //      only set to TRUE when inserting edgeuses into the subdivision Tree
   SmMemBlockMgr  *pBezierBlock)           // in : scratch memory to hold Bezier patches during construction

{
  // check state - node is from a surface subdivision tree and is not yet a parent
  SM_ASSERT(   pNode->m_eAuxDataType == SM_AD_BEZIER_SURFACE
            || pNode->m_eAuxDataType == SM_AD_AUX_DATA);
  SM_ASSERT(   pNode->m_pChild1 == NULL
            && pNode->m_pChild2 == NULL);
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
#endif // SM_DEBUG_CODE

  // locals
  SmBezierAux2d    * pAux=NULL, * pAux1=NULL, * pAux2=NULL;
  gw_SURFACE       * pSur=NULL, * pSur1=NULL, * pSur2=NULL;
  SmBezierPatch    * pBez=NULL, * pBez1=NULL, * pBez2=NULL;
  SmBSplineSurface * pSubdivisionSurface = NULL;
  SmTree           * pTree               = GetTree();
  SmSurfaceCache   * pSurfaceCache       = this;

  // don't try to split BezierPatch data for things that don't have Bezier patches
  if (pNode->m_eAuxDataType == SM_AD_AUX_DATA) 
    {
      bOnlySubdivideAuxData = TRUE;
    }

  // allocate and initialize child1 node memory    
  SmTreeNode *pNode1 = (SmTreeNode*)pTree->m_sNodeMgr.GetNewElement();
  pNode1->m_pTree   = pTree ;
  pNode1->m_pChild1 = NULL ;
  pNode1->m_pChild2 = NULL ;

  // allocate and initialize child2 node memory    
  SmTreeNode *pNode2 = (SmTreeNode*)pTree->m_sNodeMgr.GetNewElement();
  pNode2->m_pTree   = pTree ;
  pNode2->m_pChild1 = NULL ;
  pNode2->m_pChild2 = NULL ;

  // set parent/child pointer relationships between nodes
  pNode->m_pChild1  = pNode1 ;
  pNode->m_pChild2  = pNode2 ;
  pNode1->m_pParent = pNode ;
  pNode2->m_pParent = pNode ;

  // next: allocate and intialize SmTreeNodeData memory for each new child node.  
  // These will be either SmBezierPatch or a SmBezierAux2d memory objects.

  // when node has a Bezier patch to split 
  // and we want to split it - 
  // allocate and init (don't set) the SmBezierPatch memory 
  if (   pNode->m_eAuxDataType == SM_AD_BEZIER_SURFACE
      && bOnlySubdivideAuxData == FALSE) 
    {
      SmBezierPatch *pBezPatch = (SmBezierPatch*)pNode->m_pData;
      pBez = pBezPatch;
      pAux = pBezPatch;

#ifdef SM_DEBUG_CODE
      if (bDebugMe) 
        {
          SmTArray<SmTreeNode*> sNodes;
          pTree->GetAllTreeNodes(sNodes);

          pTree->Dump() ;

          smgfx_Erase();
          smgfx_SetLook(1,2, 1,0,0); pAux->m_sUVDomain.Draw(GetContext()); sm_GraphicsLoop();
          smgfx_SetLook(1,2, 0,0,0); for (ULONG j=0; j<sNodes.GetSize(); j++) 
                                       {
                                         SmTreeNode *pTreeNode = sNodes[j];
                                         if (pTreeNode->m_pChild1 != NULL) continue;
                                         SmBezierAux2d *pNodeAux = (SmBezierAux2d*)pTreeNode->m_pData;
                                         if (pNodeAux) 
                                           {
                                             pNodeAux->m_sUVDomain.Draw(GetContext()); sm_GraphicsLoop();
                                           }
                                       }
          sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE

      // If make it here then do subdivision

      // Get Surface Bezier Shape
      pSur = pBezPatch->GetBezierPtr();
      
      // Prepare Child1 BezierPatch surface memory and store it in pNode1 
      pBez1 = (SmBezierPatch*)pSurfaceCache->m_sBezMgr.GetNewElement() ;
      NER( pBez1 );
      pBez1->m_pBezier = (gw_SURFACE *)pBezierBlock->GetNewElement() ;
      NER( pBez1->m_pBezier );
      pAux1 = pBez1;
      pSur1 = pBez1->GetBezierPtr();
      
      // init internal pointers in already allocated nurb surface memory block
      sm_InitNurbSurfaceMemory(pSur1,
                               pSur->net->n, pSur->net->m,
                               pSur->p, pSur->q,
                               pSur->knu->m, pSur->knv->m);
      pNode1->m_eAuxDataType = SM_AD_BEZIER_SURFACE;
      pNode1->m_pData        = pBez1;
      pNode1->m_eGeomType    = SM_NG_DEFAULT;
      
      // Prepare Child2 BezierPatch surface memory and store it in pNode2
      pBez2 = (SmBezierPatch*)pSurfaceCache->m_sBezMgr.GetNewElement();
      NER( pBez2 );
      pBez2->m_pBezier = (gw_SURFACE *)pBezierBlock->GetNewElement() ;
      NER( pBez2->m_pBezier );
      pAux2 = pBez2;
      pSur2 = pBez2->GetBezierPtr();

      // init internal pointers in already allocated nurb surface memory block
      sm_InitNurbSurfaceMemory(pSur2,
                               pSur->net->n, pSur->net->m,
                               pSur->p, pSur->q,
                               pSur->knu->m, pSur->knv->m);
      pNode2->m_eAuxDataType = SM_AD_BEZIER_SURFACE;
      pNode2->m_pData        = pBez2;
      pNode2->m_eGeomType    = SM_NG_DEFAULT;

      // init the new BezierPatch continuity values
      pBez1->m_eUCurveConts[0] = pBezPatch->m_eUCurveConts[0];
      pBez1->m_eUCurveConts[1] = pBezPatch->m_eUCurveConts[1];
      pBez1->m_eVCurveConts[0] = pBezPatch->m_eVCurveConts[0];
      pBez1->m_eVCurveConts[1] = pBezPatch->m_eVCurveConts[1];

      pBez2->m_eUCurveConts[0] = pBezPatch->m_eUCurveConts[0];
      pBez2->m_eUCurveConts[1] = pBezPatch->m_eUCurveConts[1];
      pBez2->m_eVCurveConts[0] = pBezPatch->m_eVCurveConts[0];
      pBez2->m_eVCurveConts[1] = pBezPatch->m_eVCurveConts[1];
    } // end need to split bezier patch check

  else // not splitting bezier patch (either no bezier to split or inserting edgeuses) - just aux data
       // allocate and init (don't set) SmBezierAux2d memory.
    {
      pBez = NULL ;
      pAux = (SmBezierAux2d*)pNode->m_pData;

      // depending on history mBA_pSurface may be in one of 3 states:
      //  1. NULL
      //  2. equal to a subdivision surface the size of pNode
      //  3. equal to the root surface.
      // Remember case 2.  Later we will subdivide that surface
      //  and use the pieces to make right-sized bounding boxes.
      if (   pAux->mBA_pSurface
          && (   pNode == pTree->GetTopNode()
              || this->GetSurface() != pAux->mBA_pSurface))
        {
          // arrive here when:
          // if(  !(   pNode->m_eAuxDataType == SM_AD_BEZIER_SURFACE
          //        && bOnlySubdivideAuxData == FALSE)
          //    && (   pAux->mBA_pSurface                             
          //        && (   pNode == pTree->GetTopNode()               
          //            || &this->GetSurface() != pAux->mBA_pSurface)))
          //
          pSubdivisionSurface = (SmBSplineSurface*)pAux->mBA_pSurface;       
        }

      pAux1 = (SmBezierAux2d*)pSurfaceCache->m_sAuxMgr.GetNewElement();
      pNode1->m_eAuxDataType = SM_AD_AUX_DATA;
      pNode1->m_pData        = pAux1;
      pNode1->m_eGeomType    = SM_NG_DEFAULT;
      
      pAux2 = (SmBezierAux2d*)pSurfaceCache->m_sAuxMgr.GetNewElement();
      pNode2->m_eAuxDataType = SM_AD_AUX_DATA;
      pNode2->m_pData        = pAux2;
      pNode2->m_eGeomType    = SM_NG_DEFAULT;

    } // end init of SmBezierPatch or SmBezierAux2d memore branches

  // arrive here after allocating and initing new child node's SmTreeNodeData objects
  //   of type SmBezierAux2d or SmBezierPatch

  // set children SmBezierAux2d remaining data 
  //   - except m_eSplitDir, m_sPolarBox
  //            and all SmBezierPatch data for SmBezierPatch cases
  pAux1->m_vVertexList.Init();
  pAux1->m_sEdgeuseList.Init();
  pAux1->m_sPolyEdgeList.Init();
  pAux1->m_pOwningTreeNode = pNode1;
  pAux1->m_bIsMarked  = FALSE;
  pAux1->m_bIsTessOnly = FALSE;
  pAux1->m_eNodeClass = pAux->m_eNodeClass; // gwc: for SM_NC_BOUNDARY - need to move members of m_sPolyEdgeList, m_sEdgeuseList, m_vVertexList from parent to appropriate child
  pAux1->mBA_pSurface = NULL;
  pAux1->m_pFace      = NULL;
  pAux1->m_bChordHeightSatisfied = pAux->m_bChordHeightSatisfied;
  pAux1->m_bAngleTolSatisfied    = pAux->m_bAngleTolSatisfied;

  pAux2->m_vVertexList.Init();
  pAux2->m_sEdgeuseList.Init();
  pAux2->m_sPolyEdgeList.Init();
  pAux2->m_pOwningTreeNode = pNode2;
  pAux2->m_bIsMarked  = FALSE;
  pAux2->m_bIsTessOnly = FALSE;
  pAux2->m_eNodeClass = pAux->m_eNodeClass; // gwc: for SM_NC_BOUNDARY - need to move members of m_sPolyEdgeList, m_sEdgeuseList, m_vVertexList from parent to appropriate child
  pAux2->mBA_pSurface = NULL;
  pAux2->m_pFace      = NULL;
  pAux2->m_bChordHeightSatisfied = pAux->m_bChordHeightSatisfied;
  pAux2->m_bAngleTolSatisfied    = pAux->m_bAngleTolSatisfied;

  // Parent UV domain corners
  SmPoint2d sUVMin = pAux->m_sUVDomain.GetMin();
  SmPoint2d sUVMax = pAux->m_sUVDomain.GetMax();

  // branch on split direction SM_SP_U or SM_SP_V
  //  to build split child shapes and set child UVDomains and associated bounding boxes
  gw_FLAG dir;
  gw_PARAMETER dSplitParam;
  if (eSubdivideDirection == SM_SP_U) 
    {
      // let child1 = left
      //     child2 = right

      dir = NL_UDIR;
      pAux->m_eSplitDir = SM_SP_U;
      dSplitParam = ( sUVMax.x + sUVMin.x ) / 2.0;

      // when splitting the bezierPatch - mark child continuities across the split
      if (   pNode->m_eAuxDataType == SM_AD_BEZIER_SURFACE
          && bOnlySubdivideAuxData == FALSE) 
        {
          pBez1->m_eUCurveConts[1] = SM_CT_C1_C2;
          pBez2->m_eUCurveConts[0] = SM_CT_C1_C2;
        }

      // when just subdividing nodes (not subdividing the bezier patch)
      // and the node has a pointer to a surface
      if (pSubdivisionSurface)  // pSubdivisionSurface != NULL when     
                                //    (  !(   pNode->m_eAuxDataType == SM_AD_BEZIER_SURFACE
                                //         && bOnlySubdivideAuxData == FALSE)
                                //     && (   pAux->mBA_pSurface                             
                                //         && (   pNode == pTree->GetTopNode()               
                                //             || &this->GetSurface() != pAux->mBA_pSurface)))
        {
          // if possible move the split parameter to a nearby existing knot 
          SmPoint2d sUVOrig(dSplitParam,sUVMin.y);
          SmPoint2d sUVSnapped;
          double dSnapDist = (sUVMax.x - sUVMin.x) / 4.0;
          SER(pSubdivisionSurface->SnapToKnots(sUVOrig,dSnapDist,sUVSnapped));
          dSplitParam = sUVSnapped.x;

          // build two new split child surfaces from the original - don't modify the original
          const SmContext *pContext = pSubdivisionSurface->GetContext();
          SmSurface *pLeftBSS, *pRightBSS;
          SER(pSubdivisionSurface->SplitAt(*pContext,dSplitParam,SM_SP_U,pLeftBSS,pRightBSS));

          // store the child surfaces in the SmSurfaceCache Subdivision Surface list
          m_sSubdivisionSurfaces.Add(pLeftBSS);
          m_sSubdivisionSurfaces.Add(pRightBSS);

          // store the child surfaces in the node Surface pointers
          // gwc:note - the SM_CAST_PTR macro makes sure that only
          //              SmBrepSurfaces are stored.  If you happen
          //              to be tessellating a offset surface then the
          //              children mBA_pSurface pointers are set to NULL.
          pAux1->mBA_pSurface = SM_CAST_PTR(SmBSplineSurface,pLeftBSS);
          pAux2->mBA_pSurface = SM_CAST_PTR(SmBSplineSurface,pRightBSS);

          // Set child pAux->UVDomains
          pAux1->m_sUVDomain.SetMinMax(sUVMin,SmPoint2d(dSplitParam,sUVMax.y));
          pAux2->m_sUVDomain.SetMinMax(SmPoint2d(dSplitParam,sUVMin.y),sUVMax);

          // set pNode Bounding boxes
          pLeftBSS->CalculateBoundingBox(pAux1->m_sUVDomain, &pNode1->m_sBBox, NULL, NULL, NULL, NULL);
          pRightBSS->CalculateBoundingBox(pAux2->m_sUVDomain, &pNode2->m_sBBox, NULL, NULL, NULL, NULL);

          SM_ASSERT_MSG(pNode1->m_sBBox.IsContainedBy(pNode->m_sBBox, SM_EFF_ZERO_SQRT),_T("pChild1 BBox not contained in pParent BBox")) ;
          SM_ASSERT_MSG(pNode2->m_sBBox.IsContainedBy(pNode->m_sBBox, SM_EFF_ZERO_SQRT),_T("pChild2 BBox not contained in pParent BBox")) ;

          //      // polar boxes don't have to nest because they each have their own coordinate system
          //      // but as a rule child polar boxes should be smaller or equal than parent boxes
          //      double dPolarThisArea   = pAux->GetPolarBox().GetPolarArea() ;
          //      double dPolar1Area      = pAux1->GetPolarBox().GetPolarArea() ;
          //      double dPolar2Area      = pAux2->GetPolarBox().GetPolarArea() ;
          //      
          //      SM_ASSERT(dPolar1Area      <= dPolarThisArea + SM_EFF_ZERO_RAD) ;
          //      SM_ASSERT(dPolar2Area      <= dPolarThisArea + SM_EFF_ZERO_RAD) ;

        }
      else // is either - an SM_AD_BEZIER_SURFACE with a Bezier patch or
           //           - or embedding a Edgeuse - no geometry but get uvBox and bounding box
        {
          // Set child pAux->UVDomains
          pAux1->m_sUVDomain.SetMinMax(sUVMin,SmPoint2d(dSplitParam,sUVMax.y));
          pAux2->m_sUVDomain.SetMinMax(SmPoint2d(dSplitParam,sUVMin.y),sUVMax);

          // compute bounding box for SM_AD_AUX_DATA as best as we can
          //         but not for Node types SM_AD_BEZIER_SURFACE since the bbox is computed below.
          if (   pNode->m_eAuxDataType != SM_AD_BEZIER_SURFACE
              || bOnlySubdivideAuxData == TRUE)
            {
              gw_SURFACE *pSurf = pNode->GetFirstBezierPatch() ;
              if(pSurf) { SmBSplineSurface sBSS(pSurf, TRUE, GetContext()) ;

                         // gwc TODO: use the following two lines of code when the CalculateBoundingBox
                         //           method is modified to work on subDomains - for now just copy parent boxes
                         //      sBSS.CalculateBoundingBox(pAux1->m_sUVDomain, &pNode1->m_sBBox, NULL, &pAux1->m_sPolarBox) ;
                         //      sBSS.CalculateBoundingBox(pAux2->m_sUVDomain, &pNode2->m_sBBox, NULL, &pAux2->m_sPolarBox) ;

                         pNode1->m_sBBox = pNode->m_sBBox ;
                         pNode2->m_sBBox = pNode->m_sBBox ;
                         //pAux1->m_sPolarBox = pAux->m_sPolarBox ;
                         //pAux2->m_sPolarBox = pAux->m_sPolarBox ;

                         SM_ASSERT_MSG(pNode1->m_sBBox.IsContainedBy(pNode->m_sBBox, SM_EFF_ZERO_SQRT),_T("pChild1 BBox not contained in parent BBox")) ;
                         SM_ASSERT_MSG(pNode2->m_sBBox.IsContainedBy(pNode->m_sBBox, SM_EFF_ZERO_SQRT),_T("pChild2 BBox not contained in parent BBox")) ;

                         //      // polar boxes don't have to nest because they each have their own coordinate system
                         //      // but as a rule child polar boxes should be smaller or equal than parent boxes
                         //      double dPolarThisArea   = pAux->GetPolarBox().GetPolarArea() ;
                         //      double dPolar1Area      = pAux1->GetPolarBox().GetPolarArea() ;
                         //      double dPolar2Area      = pAux2->GetPolarBox().GetPolarArea() ;
                         //      
                         //      SM_ASSERT(dPolar1Area      <= dPolarThisArea + SM_EFF_ZERO_RAD) ;
                         //      SM_ASSERT(dPolar2Area      <= dPolarThisArea + SM_EFF_ZERO_RAD) ;
                       }
              else     {
                         // arrive here with no 3d information to build BBoxes.  Just copy Parent boxes.
                         //pAux1->m_sPolarBox = pAux->m_sPolarBox ; 
                         //pAux2->m_sPolarBox = pAux->m_sPolarBox ;
                         pNode1->m_sBBox = pNode->m_sBBox ;
                         pNode2->m_sBBox = pNode->m_sBBox ; 

                         SM_ASSERT_MSG(pNode1->m_sBBox.IsContainedBy(pNode->m_sBBox, SM_EFF_ZERO_SQRT),_T("pChild1 BBox not contained in parent BBox")) ;
                         SM_ASSERT_MSG(pNode2->m_sBBox.IsContainedBy(pNode->m_sBBox, SM_EFF_ZERO_SQRT),_T("pChild2 BBox not contained in parent BBox")) ;
                       }
            } 

        } // end no pSubdivisionSurface branch
    } // end split the u domain

  else // split the v domain
    {
      // let child1 = bot
      //     child2 = top

      dir = NL_VDIR;
      pAux->m_eSplitDir = SM_SP_V;
      dSplitParam = ( sUVMax.y + sUVMin.y ) / 2.0;

      // when splitting the bezierPatch - mark child continuities across the split
      if (   pNode->m_eAuxDataType == SM_AD_BEZIER_SURFACE
          && bOnlySubdivideAuxData == FALSE) 
        {
          pBez1->m_eVCurveConts[1] = SM_CT_C1_C2;
          pBez2->m_eVCurveConts[0] = SM_CT_C1_C2;
        }

      // when just subdividing nodes (not subdividing the bezier patch)
      // and the node has a pointer to a surface
      if (pSubdivisionSurface)  // pSubdivisionSurface != NULL when     
                                //    (  !(   pNode->m_eAuxDataType == SM_AD_BEZIER_SURFACE
                                //        && bOnlySubdivideAuxData == FALSE)
                                //    && (   pAux->mBA_pSurface                             
                                //        && (   pNode == pTree->GetTopNode()               
                                //            || &this->GetSurface() != pAux->mBA_pSurface)))
        {
          // if possible move the split parameter to a nearby existing knot 
          SmPoint2d sUVOrig(sUVMin.x,dSplitParam);
          SmPoint2d sUVSnapped;
          double dSnapDist = (sUVMax.y - sUVMin.y) / 4.0;
          SER(pSubdivisionSurface->SnapToKnots(sUVOrig,dSnapDist,sUVSnapped));
          dSplitParam = sUVSnapped.y;
          
          // build two new split child surfaces from the original - don't modify the original
          const SmContext *pContext = pSubdivisionSurface->GetContext();
          SmSurface *pBottomBSS, *pTopBSS;
          SER(pSubdivisionSurface->SplitAt(*pContext,dSplitParam,SM_SP_V,pBottomBSS,pTopBSS));

          // store the child surfaces in the SmSurfaceCache Subdivision Surface list
          m_sSubdivisionSurfaces.Add(pBottomBSS);
          m_sSubdivisionSurfaces.Add(pTopBSS);
          
          // store the child surfaces in the node Surface pointers
          pAux1->mBA_pSurface = SM_CAST_PTR(SmBSplineSurface,pBottomBSS);
          pAux2->mBA_pSurface = SM_CAST_PTR(SmBSplineSurface,pTopBSS);

          // Set child pAux->UVDomains
          pAux1->m_sUVDomain.SetMinMax(sUVMin,SmPoint2d(sUVMax.x,dSplitParam));
          pAux2->m_sUVDomain.SetMinMax(SmPoint2d(sUVMin.x,dSplitParam),sUVMax);

          // set pNode Bounding boxes
          // It's useless to continue if these fail, so SER. [B62]
          SER( pBottomBSS->CalculateBoundingBox(pAux1->m_sUVDomain,
                                                &pNode1->m_sBBox, NULL, 
                                                NULL, NULL, 
                                                NULL));
          SER( pTopBSS->CalculateBoundingBox   (pAux2->m_sUVDomain,
                                                &pNode2->m_sBBox, NULL, 
                                                NULL, NULL, 
                                                NULL));

#ifdef SM_DEBUG_CODE
          if(!pNode1->m_sBBox.IsContainedBy(pNode->m_sBBox, SM_EFF_ZERO_SQRT))
            { SM_ASSERT_MSG(pNode1->m_sBBox.IsContainedBy(pNode->m_sBBox, SM_EFF_ZERO_SQRT),_T("pChild1 BBox not contained in parent BBox")) ; }
          if(!pNode2->m_sBBox.IsContainedBy(pNode->m_sBBox, SM_EFF_ZERO_SQRT))
            { SM_ASSERT_MSG(pNode2->m_sBBox.IsContainedBy(pNode->m_sBBox, SM_EFF_ZERO_SQRT),_T("pChild2 BBox not contained in parent BBox")) ; }

          //      // polar boxes don't have to nest because they each have their own coordinate system
          //      // but as a rule child polar boxes should be smaller or equal than parent boxes
          //      double dPolarThisArea   = pAux->GetPolarBox().GetPolarArea() ;
          //      double dPolar1Area      = pAux1->GetPolarBox().GetPolarArea() ;
          //      double dPolar2Area      = pAux2->GetPolarBox().GetPolarArea() ;
          //      
          //      SM_ASSERT(dPolar1Area      <= dPolarThisArea + SM_EFF_ZERO_RAD) ;
          //      SM_ASSERT(dPolar2Area      <= dPolarThisArea + SM_EFF_ZERO_RAD) ;

#endif // SM_DEBUG_CODE

        }
      else // is either - an SM_AD_BEZIER_SURFACE with a Bezier patch or
           //           - or embedding a Edgeuse - no geometry but get uvBox and bounding box
        {
          // Set child pAux->UVDomains
          pAux1->m_sUVDomain.SetMinMax(sUVMin,SmPoint2d(sUVMax.x,dSplitParam));
          pAux2->m_sUVDomain.SetMinMax(SmPoint2d(sUVMin.x,dSplitParam),sUVMax);

          // compute bounding box for SM_AD_AUX_DATA as best as we can
          //         but not for Node types SM_AD_BEZIER_SURFACE since the bbox is computed below.
          if (   pNode->m_eAuxDataType != SM_AD_BEZIER_SURFACE
              || bOnlySubdivideAuxData == TRUE)
            {
              gw_SURFACE *pSurf = pNode->GetFirstBezierPatch() ;
              if(pSurf) { SmBSplineSurface sBSS(pSurf, TRUE, GetContext()) ;

                         // gwc TODO: use the following two lines of code when the CalculateBoundingBox
                         //           method is modified to work on subDomains - for now just copy parent boxes
                         //      sBSS.CalculateBoundingBox(pAux1->m_sUVDomain, &pNode1->m_sBBox, NULL, &pAux1->m_sPolarBox) ;
                         //      sBSS.CalculateBoundingBox(pAux2->m_sUVDomain, &pNode2->m_sBBox, NULL, &pAux2->m_sPolarBox) ;

                         pNode1->m_sBBox = pNode->m_sBBox ;
                         pNode2->m_sBBox = pNode->m_sBBox ;
                         //pAux1->m_sPolarBox = pAux->m_sPolarBox ;
                         //pAux2->m_sPolarBox = pAux->m_sPolarBox ;

                         SM_ASSERT_MSG(pNode1->m_sBBox.IsContainedBy(pNode->m_sBBox, SM_EFF_ZERO_SQRT),_T("pChild1 BBox not contained in parent BBox")) ;
                         SM_ASSERT_MSG(pNode2->m_sBBox.IsContainedBy(pNode->m_sBBox, SM_EFF_ZERO_SQRT),_T("pChild2 BBox not contained in parent BBox")) ;
                         //      // polar boxes don't have to nest because they each have their own coordinate system
                         //      // but as a rule child polar boxes should be smaller or equal than parent boxes
                         //      double dPolarThisArea   = pAux->GetPolarBox().GetPolarArea() ;
                         //      double dPolar1Area      = pAux1->GetPolarBox().GetPolarArea() ;
                         //      double dPolar2Area      = pAux2->GetPolarBox().GetPolarArea() ;
                         //      
                         //      SM_ASSERT(dPolar1Area      <= dPolarThisArea + SM_EFF_ZERO_RAD) ;
                         //      SM_ASSERT(dPolar2Area      <= dPolarThisArea + SM_EFF_ZERO_RAD) ;
                       }
              else     { 
                         // arrive here when nodes are being subdivided to accomodate UVTrimCurve additions
                         // after the temporary hierarchy of subdivision surfaces have been freed.

                         // The best we can do now is either
                         // 1. copy the parent Bounding Box data or
                         // 2. Modify the CalculateBoundingBox to actually work on surface subDomains
                         pNode1->m_sBBox = pNode->m_sBBox ;
                         pNode2->m_sBBox = pNode->m_sBBox ;
                         //pAux1->m_sPolarBox = pAux->m_sPolarBox ; 
                         //pAux2->m_sPolarBox = pAux->m_sPolarBox ;

                         SM_ASSERT_MSG(pNode1->m_sBBox.IsContainedBy(pNode->m_sBBox, SM_EFF_ZERO_SQRT),_T("pChild1 BBox not contained in parent BBox")) ;
                         SM_ASSERT_MSG(pNode2->m_sBBox.IsContainedBy(pNode->m_sBBox, SM_EFF_ZERO_SQRT),_T("pChild2 BBox not contained in parent BBox")) ;
                         SM_ASSERT(pNode1->m_eAuxDataType == SM_AD_AUX_DATA) ;
                         SM_ASSERT(pNode2->m_eAuxDataType == SM_AD_AUX_DATA) ;
                       }
            } 
        }
    } // end split v domain branch

#ifdef SM_DEBUG_CODE
  if (bDebugMe) 
    {
      pTree->Dump() ;
      pNode->DumpFamily() ;

      const SmSurface *cpSurface = GetSurface() ;
      SmPoint3d sCenter, sCenter1, sCenter2 ;
      cpSurface->EvaluatePoint(pAux->m_sUVDomain.Evaluate(.5,.5),  sCenter) ;
      cpSurface->EvaluatePoint(pAux1->m_sUVDomain.Evaluate(.5,.5), sCenter1) ;
      cpSurface->EvaluatePoint(pAux2->m_sUVDomain.Evaluate(.5,.5), sCenter2) ;

      smgfx_Erase();
      smgfx_SetLook(1,2, 1,0,0) ; if(pSubdivisionSurface) pSubdivisionSurface->DrawUV(4,4,FALSE,&pAux->m_sUVDomain) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,0) ; if(pAux1->mBA_pSurface) pAux1->mBA_pSurface->DrawUV(4,4,FALSE,&pAux->m_sUVDomain) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pAux2->mBA_pSurface) pAux2->mBA_pSurface->DrawUV(4,4,FALSE,&pAux->m_sUVDomain) ; sm_GraphicsLoop() ;

      smgfx_SetLook(1,2, 1,0,0) ; cpSurface->DrawUV(3,3,FALSE,&pAux->m_sUVDomain); sm_GraphicsLoop();
      smgfx_SetLook(1,2, 1,0,1) ; pNode->m_sBBox.Draw(); sm_GraphicsLoop() ;
      //smgfx_SetLook(1,2, 1,0,1) ; pAux->m_sPolarBox.Draw(sCenter) ; sm_GraphicsLoop() ;
      //smgfx_SetLook(1,2, 0,1,1) ; pAux1->m_sPolarBox.Draw(sCenter1) ; sm_GraphicsLoop() ;
      //smgfx_SetLook(1,2, 1,1,0) ; pAux2->m_sPolarBox.Draw(sCenter2) ; sm_GraphicsLoop() ;

      smgfx_SetLook(1,2, 0,1,0) ; cpSurface->DrawUV(3,3,FALSE,&pAux1->m_sUVDomain); sm_GraphicsLoop();
      smgfx_SetLook(1,2, 0,1,1) ; pNode1->m_sBBox.Draw(); sm_GraphicsLoop() ;

      smgfx_SetLook(1,2, 0,0,1) ; cpSurface->DrawUV(3,3,FALSE,&pAux2->m_sUVDomain); sm_GraphicsLoop();
      smgfx_SetLook(1,2, 1,1,0) ; pNode2->m_sBBox.Draw(); sm_GraphicsLoop() ;

      smgfx_SetRotationCenter(sCenter) ;
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE
  
  // still need BoundingBoxes when node has a Bezier patch to split
  // or   surface is other than SmBSPlineSurface and we are building approximate shapes at each node
  // and we want to split it
  if (   pNode->m_eAuxDataType == SM_AD_BEZIER_SURFACE
      && bOnlySubdivideAuxData == FALSE) 
    {
      // arrive here after setting only
      //     pAux->M_eSplitDir                                                 
      //     pBez1->m_eVCurveConts                                             
      //     pBez2->m_eVCurveConts                                             
      //     pAux1->m_sUVDomain
      //     pAux2->m_sUVDomain
      // need to set
      //     pNode1->m_sBBox,        pNode2->m_sBBox,    
      //     pBez1->m_sPseudoBox,    pBez2->m_sPseudoBox,
      //     pBez1->m_sPolarBox,     pBez2->m_sPolarBox, 
            
      // when caching a BSplineSurface
      //      let children surfaces = split Parent Surface
      //SmBoolean bBSplineTYPE = m_cpSurface->IsKindOf(SmBSplineSurface_TYPE) ;
      //SM_TYPE   lType        = m_cpSurface->GetType() ;
      if (m_cpSurface->IsKindOf(SmBSplineSurface_TYPE)) 
        {
          // build child GW_SURFACEs = split parent BezierPatch Surface
          GW_ERR(sm_SplitSrf(pSur,        // in : tgt surface
                             dSplitParam, // in : split param
                             dir,         // in : SplitDir: NL_UDIR=Split KnotVectorU - use: SM_SURFPARAM_TO_NLDIR(eSurfParam)
                                          //                NL_VDir=Split KnotVectorV        to convert SmSurfParamType to NL_DIR types
                             pSur1,       // out: Split surface result, Ivl=[MinParam, TgtParam]
                             pSur2));     // out: Split surface result, Ivl=[TgtParam, MaxParam]
        }
      else // when caching a non-SmBSplineSurface type surface
           // let child BezierPatch surfaces = Bezier approximations to Parent Surface subdomain
           //   to keep approximation errors small. 
           // note: except for the leaf nodes, the approximation errors
           //       will all be large.  But these will be used to determine
           //       if subsequent subdivision is required.
        {
          // let child BezierPatch surfaces = parent approximations.
          //  These are very coarse approximations for all but the final leaf nodes.
          pSur1 = pBez1->GetBezierPtr();
          sm_ComputeBezierApproximation(*m_cpSurface,pAux1->m_sUVDomain,pSur1);

          gw_SURFACE *pSurf2 = pBez2->GetBezierPtr();
          sm_ComputeBezierApproximation(*m_cpSurface,pAux2->m_sUVDomain,pSurf2);
        }
  
      // Get and store child 1 and 2 Bounding and Pseudo boxes
      SmBSplineSurface sBSS1(pSur1,TRUE,m_cpSurface->GetContext()) ;
      SmBSplineSurface sBSS2(pSur2,TRUE,m_cpSurface->GetContext()) ;
      // sBSS1.SetContext(m_cpSurface->GetContext());
      // sBSS2.SetContext(m_cpSurface->GetContext());

      SER(sBSS1.CalculateBoundingBox(sBSS1.GetNaturalUVDomain(), &pNode1->m_sBBox, &pBez1->m_sPseudoBox, NULL, &pBez->GetPseudoBox(), NULL));

      // Get and store child 2 Bounding and Pseudo boxes
      SER(sBSS2.CalculateBoundingBox(sBSS2.GetNaturalUVDomain(), &pNode2->m_sBBox, &pBez2->m_sPseudoBox, NULL, &pBez->GetPseudoBox(), NULL));

#ifdef SM_DEBUG_CODE
      if(   !m_cpSurface->IsKindOf(SmBSplineSurface_TYPE)
         && (   !pNode1->m_sBBox.IsContainedBy(pNode->m_sBBox, SM_EFF_ZERO_SQRT)
             || !pNode2->m_sBBox.IsContainedBy(pNode->m_sBBox, SM_EFF_ZERO_SQRT)))
        {
          SM_ASSERT_MSG(   !m_cpSurface->IsKindOf(SmBSplineSurface_TYPE)
                        || pNode1->m_sBBox.IsContainedBy(pNode->m_sBBox, SM_EFF_ZERO_SQRT),_T("pChild1 BBox not contained in Parent BBox")) ;
          SM_ASSERT_MSG(!m_cpSurface->IsKindOf(SmBSplineSurface_TYPE)                          
                        || pNode2->m_sBBox.IsContainedBy(pNode->m_sBBox, SM_EFF_ZERO_SQRT),_T("pChild2 BBox not contained in Parent BBox")) ;
        }
#endif // SM_DEBUG_CODE

      //      // polar boxes don't have to nest because they each have their own coordinate system
      //      // but as a rule child polar boxes should be smaller or equal than parent boxes
      //      double dPolarThisArea   = pBez->GetPolarBox().GetPolarArea() ;
      //      double dPolar1Area      = pBez1->GetPolarBox().GetPolarArea() ;
      //      double dPolar2Area      = pBez2->GetPolarBox().GetPolarArea() ;
      //      
      //      if(   dPolar1Area  > dPolarThisArea + SM_EFF_ZERO_RAD
      //         || dPolar2Area  > dPolarThisArea + SM_EFF_ZERO_RAD)
      //        {
      //          SM_ASSERT(dPolar1Area      <= dPolarThisArea + SM_EFF_ZERO_RAD) ;
      //          SM_ASSERT(dPolar2Area      <= dPolarThisArea + SM_EFF_ZERO_RAD) ;
      //        }

      // gwc:SHORTCACHE
      //      // Center Point Evaluation for children patch 1
      //      SmVector3d sEval1[2][2] ;
      //      SmPoint2d sUVMid1   = pBez1->m_sUVDomain.Evaluate(0.5,0.5);
      //      SER(sBSS1.Evaluate(sUVMid1,1,1,TRUE,TRUE,TRUE,sEval1[0])); // gwc: sBSS.Evaluate(sUVMid,2,2,TRUE,TRUE,TRUE,sEval[0])
      //      pBez1->m_sMidPoint  = sEval1[0][0];
      //      pBez1->m_sMidDU     = sEval1[1][0];
      //      pBez1->m_sMidDV     = sEval1[0][1];
      //      pBez1->m_sMidNormal = pBez1->m_sMidDU * pBez1->m_sMidDV;
      //      if (pBez1->m_sMidNormal.LengthSquared() > SM_EFF_ZERO) { SER(pBez1->m_sMidNormal.Unitize()); }
      //      else                                                   { SER(sBSS1.EvaluateNormal(sUVMid1,TRUE,TRUE,pBez1->m_sMidNormal)); }
      //      
      //      // Center Point Evaluation for children patch 2
      //      SmVector3d sEval2[2][2];
      //      SmPoint2d sUVMid2   = pBez2->m_sUVDomain.Evaluate(0.5,0.5);
      //      SER(sBSS2.Evaluate(sUVMid2,1,1,TRUE,TRUE,TRUE,sEval2[0])); // gwc: sBSS.Evaluate(sUVMid,2,2,TRUE,TRUE,TRUE,sEval[0])
      //      pBez2->m_sMidPoint  = sEval2[0][0];
      //      pBez2->m_sMidDU     = sEval2[1][0];
      //      pBez2->m_sMidDV     = sEval2[0][1];
      //      pBez2->m_sMidNormal = pBez2->m_sMidDU * pBez2->m_sMidDV;
      //      if (pBez2->m_sMidNormal.LengthSquared() > SM_EFF_ZERO) { SER(pBez2->m_sMidNormal.Unitize()); }
      //      else                                                   { SER(sBSS2.EvaluateNormal(sUVMid2,TRUE,TRUE,pBez2->m_sMidNormal)); }
                                                             
#ifdef SM_DEBUG_CODE
      if (bDebugMe) 
        {
          SmBSplineSurface sBSS(pSur,TRUE,m_cpSurface->GetContext()) ;
          // sBSS.SetContext(m_cpSurface->GetContext());
          SmPoint3d sCenter, sCenter1, sCenter2 ;
          m_cpSurface->EvaluatePoint(pAux->GetUVDomain().Evaluate(.5,.5),  sCenter) ;
          m_cpSurface->EvaluatePoint(pAux1->GetUVDomain().Evaluate(.5,.5), sCenter1) ;
          m_cpSurface->EvaluatePoint(pAux2->GetUVDomain().Evaluate(.5,.5), sCenter2) ;

          m_cpSurface->Dump() ;
          if(pSur) sBSS.Dump() ;
          sBSS1.Dump();
          sBSS2.Dump();
          if(pNode) pNode->Dump(); // dumps node and children

          smgfx_Erase();
          smgfx_SetLook(1,2, 1,1,0) ;  if(m_cpSurface) m_cpSurface->DrawUV(4,4);                            sm_GraphicsLoop();
          smgfx_SetLook(1,2, 1,.5,0) ; if(m_cpSurface) m_cpSurface->DrawUV(5,5,FALSE,&pAux->GetUVDomain()); sm_GraphicsLoop();


          smgfx_SetLook(1,2, 0,1,1) ;                                                 
                                      if(pSur) {sBSS.DrawUV(3,3);                      sm_GraphicsLoop();}
                                      pNode->GetBoundingBox().Draw() ;                sm_GraphicsLoop() ;
                                      if(pAux) { pAux->GetPolarBox().Draw( sCenter );    sm_GraphicsLoop(); }
                                      if(pBez) { pBez->GetPseudoBox().Draw();     sm_GraphicsLoop(); }
          smgfx_SetLook(1,2, 0,1,0) ; 
                                      sBSS1.DrawUV(4,4);                              sm_GraphicsLoop();
                                      sBSS1.DrawNet();                                sm_GraphicsLoop();
                                      pNode1->GetBoundingBox().Draw() ;               sm_GraphicsLoop() ;
                                      if(pAux1) { pAux1->GetPolarBox().Draw( sCenter1 ); sm_GraphicsLoop(); }
                                      if(pBez1) { pBez1->GetPseudoBox().Draw();  sm_GraphicsLoop(); }
          smgfx_SetLook(1,2, 0,0,1) ; 
                                      sBSS2.DrawUV(4,4);                              sm_GraphicsLoop();
                                      sBSS2.DrawNet();                                sm_GraphicsLoop();
                                      pNode2->GetBoundingBox().Draw() ;               sm_GraphicsLoop() ;
                                      if(pAux2) { pAux2->GetPolarBox().Draw( sCenter2 ); sm_GraphicsLoop(); }
                                      if(pBez2) { pBez2->GetPseudoBox().Draw();  sm_GraphicsLoop(); }
          smgfx_SetRotationCenter(sCenter) ;
          sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE

    } // end subdivide a SM_AD_BEZIER_SURFACE node with a SmBezierPatch object check

  // Now update the TreeVertex Model
    {
      // locals
      // ULONG i0, i1, j0, j1 ;
      // ULONG lIncTreeVertexCount ;
      SmTArray<double> sUSplits, sVSplits ;
      SmTArray<ULONG> sTreeVertexMap, sTreeVertexPre, sTreeVertexPost ;

      // build the split arrays - remember the Starting Loop index for both new Nodes
      sUSplits.Add(sUVMin.x) ;       
      sVSplits.Add(sUVMin.y) ;       
      if (eSubdivideDirection == SM_SP_U) { sUSplits.Add(dSplitParam) ;
                                            //i0 = 0 ; j0 = 1 ;  // let child1 = left
                                            //i1 = 1 ; j1 = 1 ;  //     child2 = right
                                          }
      else                                { sVSplits.Add(dSplitParam) ;
                                            //i0 = 0 ; j0 = 1 ;  // let child1 = bot
                                            //i1 = 0 ; j1 = 2 ;  //     child2 = top
                                          }
      sUSplits.Add(sUVMax.x) ;
      sVSplits.Add(sUVMax.y) ;

// GWCTreeVertexTemp
//      // Build the TreeVertexMaps - increment m_sTreeVertices as needed
//      pTree->BuildTreeVertexMaps
//        (pNode,              // in : Parent Node being split before TreeVertices entries are updated or NULL for none
//         sUSplits,           // in : rUSplits[i] = ith Parent Node Split Point with bndrys. Size of 2=no splits, 3=1 split, . . .
//         sVSplits,           // in : rVSplits[j] = jth Parent Node Split Point with bndrys. Size of 2=no splits, 3=1 split, . . .
//         sTreeVertexMap,     // out: Index for every U/V split TreeVertex intersection
//         sTreeVertexPre,     // out: closest TreeVertexIndex to This between This and Last U/V SplitPoints, 
//                             //      SM_BIG_ULONG = no TreeVertices between This and Last U/V SplitPoint
//         sTreeVertexPost) ;  // out: closest TreeVertexIndex to This between This and Next U/V SplitPoints, 
//                             //      SM_BIG_ULONG = no TreeVertices between This and Next U/V SplitPoint

// GWCTreeVertexTemp
//  #ifdef SM_DEBUG_CODE
//        if (bDebugMe) 
//          { 
//            SmBoolean bOk = pTree->AssertTreeVertices() ; 
//            bOk = pTree->AssertTreeVertices() ; 
//          }
//  #endif // SM_DEBUG_CODE

// GWCTreeVertexTemp
//      // create/connect 1st node's TreeVertex Loop - set leaf node Start TreeVertex Index
//      pAux1->m_lStartTreeVertexIndx = m_pTree->SetTreeVertexLoop
//                                (pNode1,                   // in : TreeNode owner of this TreeVertex Loop
//                                 pAux1->m_sUVDomain,       // in : UVDomain for Node with this loop
//                                 i0, j0,                   // in : Above Left Corner index[i,j] of new Node TreeVertex Loop
//                                 sUSplits, sVSplits,       // in : Total Number of U and V Splits in TreeVertex Array(ii,jj)
//                                 sTreeVertexMap,           // in : Array made by BuildTreeVertexMaps call
//                                 sTreeVertexPre,           // in : Array Made By BuildTreeVertexMaps call
//                                 sTreeVertexPost) ;        // in : Array Made By BuildTreeVertexMaps call
//
//      // note: VertexTree pTreeNode pointers are incompletely set until next call
//       
//      // create/connect 2nd node's TreeVertex Loop - set leaf node Start TreeVertex Index
//      pAux2->m_lStartTreeVertexIndx = m_pTree->SetTreeVertexLoop
//                                (pNode2,                   // in : TreeNode owner of this TreeVertex Loop
//                                 pAux2->m_sUVDomain,       // in : UVDomain for Node with this loop
//                                 i1, j1,                   // in : Above Left Corner index[i,j] of new Node TreeVertex Loop
//                                 sUSplits, sVSplits,       // in : Total Number of U and V Splits in TreeVertex Array(ii,jj)
//                                 sTreeVertexMap,           // in : Array made by BuildTreeVertexMaps call
//                                 sTreeVertexPre,           // in : Array Made By BuildTreeVertexMaps call
//                                 sTreeVertexPost) ;        // in : Array Made By BuildTreeVertexMaps call
//
      // note: TreeVertex structure should be completely modified now

// GWCTreeVertexTemp
//  #ifdef SM_DEBUG_CODE
//  SmBoolean bDebugAssert = FALSE ;
//         if(bDebugAssert)
//           {
//             SmBoolean bRtn = m_pTree->AssertTreeVertices() ;
//             m_pTree->DumpTreeVertices() ;
//             SM_ASSERT_BREAK(bRtn) ;
//           }
//  #endif // SM_DEBUG_CODE
//
//      // clear the parent StartTreeVertexIndx value
//      pAux->m_lStartTreeVertexIndx = SM_BIG_ULONG ; 

    } // end scope to update TreeVertex Model

  // when parentNode->pAux->m_eNodeClass == SM_NC_ON_BOUNDARY
  //   it has entries in its m_sEdgeuseList.  All the edgeuse data stored in
  //   the pNode->pAux data structure need to be removed and implanted in the
  //   children.
  //
  //   Currently - this case only happens for class SmTrimSrfCache caches
  //   before the call to 
  //   SmTrimSrfCache::ImplantVertexuses is made, so the m_vVertexList is
  //   expected to be empty.  If that changes and m_vVertexList is not
  //   empty then add code to move the vertexuses from the parent to the child
  //   in the same manner as edgeuses are being treated.
  if(pAux->m_eNodeClass == SM_NC_ON_BOUNDARY)
    {
      // init the node classification
      pAux1->m_eNodeClass = SM_NC_UNKNOWN ;
      pAux2->m_eNodeClass = SM_NC_UNKNOWN ;
      SmExtent2d sTempExt ;

      // check assumptions
      SM_ASSERT(this->IsKindOf(SmTrimSrfCache_TYPE)) ;
      SM_ASSERT(pAux1->m_vVertexList.GetFirstNode() == NULL) ;
      SM_ASSERT(pAux2->m_vVertexList.GetFirstNode() == NULL) ;

      // check to see if the PolyEdge cache is populated, if so we will embed
      // polyedge cache in boundary leaves.

      if (pAux->m_sPolyEdgeList.GetFirstNode() != NULL)
        {
          // if we have elements in polyedge cache, get them 

          SmTArray<SmPolyEdgeList*> sPolyEdgeListArray;
          pAux->m_sPolyEdgeList.GetAllNodes(sPolyEdgeListArray);

          for (ULONG ii = 0; ii < sPolyEdgeListArray.GetSize(); ii++)
            {
              SmPolyEdgeList* pPolyEdgeList = sPolyEdgeListArray[ii];
              SmPolyEdge* pPolyEdge = pPolyEdgeList->m_pPolyEdge;

              // Attempt to embed polyedge in Child1, if successful
              // we label the child as SM_NC_ON_BOUNDARY
              ImplantPolyEdge(pPolyEdge, pNode1);

              // Attempt to embed polyedge in Child2, if successful
              // we label the child as SM_NC_ON_BOUNDARY
              ImplantPolyEdge(pPolyEdge, pNode2);
            } // end for each polyedge in cache
        }
      else
        {
          // for every edgeuse in the parent node
          SmEdgeuseList* pEdgeuseListNode = NULL;
          for (pEdgeuseListNode = pAux->m_sEdgeuseList.GetLastNode(); pEdgeuseListNode != NULL;
                pAux->m_sEdgeuseList.RemoveLast(), pEdgeuseListNode = pAux->m_sEdgeuseList.GetLastNode())
            {
              SmEdgeuse* pEU = pEdgeuseListNode->m_pEU;

              // implant edgeuse in Child1 without subsequent subdivision
              ImplantEdgeuse(pEU, FALSE, sTempExt, pNode1);
                
              // implant edgeuse in Child2 without subsequent subdivision
              ImplantEdgeuse(pEU, FALSE, sTempExt, pNode2);

             } // end iter every edgeuse
        }
      // at least one of the childnodes should now be classified on the boundary
      SM_ASSERT(   pAux1->m_eNodeClass == SM_NC_ON_BOUNDARY 
                || pAux2->m_eNodeClass == SM_NC_ON_BOUNDARY) ; 

    } // end need to move edgeuses to children nodes check

  // all done
  return SM_SUCCESS;

} // end SmSurfaceCache::SubdivideNode

// GWC:removed unused function
//      
//      /*******************************************************************//**
//      PURPOSE: Subdivide the given node when the shape is a cubic bezier.
//      
//      NOTES: 
//      ***********************************************************************/
//      SmStatus SmSurfaceCache::SubdivideNodeFast
//        (SmTree *pTree,                               // in : tree being modified
//         SmTreeNode *pNode,                           // in : node in tree being split
//         SmSurfParamType eSubdivideDirection,         // in : SM_SP_U = split u domain, make left/right children
//                                                      //      SM_SP_V = split v domain, make bot/top children
//         const SmCubicBezierSurface & crOrigBez,      // in : shape to split
//         SmCubicBezierSurface & crBezier1,            // out: lower half of split input domain
//         SmCubicBezierSurface & crBezier2)            // out: upper half of split input domain
//      {
//        // check state - a surface leaf node
//        SM_ASSERT(   pNode->m_eAuxDataType == SM_AD_BEZIER_SURFACE
//                  || pNode->m_eAuxDataType == SM_AD_AUX_DATA);
//        SM_ASSERT(pNode->m_pChild1 == NULL);
//      
//        // locals
//        SmBezierAux2d  *pAux          = (SmBezierAux2d*)pNode->m_pData;
//        SmBezierAux2d  *pAux1         = NULL ;
//        SmBezierAux2d  *pAux2         = NULL ;
//        SmSurfaceCache  *pSurfaceCache = this;
//      
//        // if the node being split has no surface use the surface stored in the SmSurfaceCache
//        const SmSurface *pNodeSurface  = (pAux->mBA_pSurface) ? pAux->mBA_pSurface : m_cpSurface ;
//      
//      // SmBSplineSurface *pSubdivisionSurface = NULL;
//      
//        // allocate child 1 and set family pointers    
//        SmTreeNode *pNode1 = (SmTreeNode*)pTree->m_sNodeMgr.GetNewElement();
//        pNode1->m_pTree   = pTree ;
//        pNode1->m_pChild1 = NULL ;
//        pNode1->m_pChild2 = NULL ;
//        pNode1->m_pParent = pNode ;
//        pNode->m_pChild1  = pNode1 ;
//      
//        // allocate child 2 and set family pointers    
//        SmTreeNode *pNode2 = (SmTreeNode*)pTree->m_sNodeMgr.GetNewElement();
//        pNode2->m_pTree   = pTree;
//        pNode2->m_pChild1 = NULL ;
//        pNode2->m_pChild2 = NULL ;
//        pNode2->m_pParent = pNode ;
//        pNode->m_pChild2  = pNode2 ;
//      
//        // allocate, init, and store child1 SmBezierAux2d block
//        pAux1 = (SmBezierAux2d*)pSurfaceCache->m_sAuxMgr.GetNewElement();
//        pNode1->m_eAuxDataType         = SM_AD_AUX_DATA;
//        pNode1->m_pData                = pAux1;
//        pAux1->m_bChordHeightSatisfied = pAux->m_bChordHeightSatisfied;
//        pAux1->m_bAngleTolSatisfied    = pAux->m_bAngleTolSatisfied;
//        pAux1->m_bIsMarked             = FALSE;
//        pAux1->m_eNodeClass            = pAux->m_eNodeClass;
//        pAux1->mBA_pSurface              = pAux->mBA_pSurface;
//        pAux1->m_pFace                 = NULL;
//        pAux1->m_vVertexList.Init();
//        pAux1->m_sEdgeuseList.Init();
//        pAux1->m_sPolyEdgeList.Init();
//       
//        // allocate, init, and store child2 SmBezierAux2d block
//        pAux2 = (SmBezierAux2d*)pSurfaceCache->m_sAuxMgr.GetNewElement();
//        pNode2->m_eAuxDataType         = SM_AD_AUX_DATA;
//        pNode2->m_pData                = pAux2;
//        pAux2->m_bChordHeightSatisfied = pAux->m_bChordHeightSatisfied;
//        pAux2->m_bAngleTolSatisfied    = pAux->m_bAngleTolSatisfied;
//        pAux2->m_bIsMarked             = FALSE;
//        pAux2->m_eNodeClass            = pAux->m_eNodeClass;
//        pAux2->mBA_pSurface              = pAux->mBA_pSurface;
//        pAux2->m_pFace                 = NULL;
//        pAux2->m_vVertexList.Init();
//        pAux2->m_sEdgeuseList.Init();
//        pAux2->m_sPolyEdgeList.Init();
//      
//        // get parent UVDomain corners
//        SmPoint2d sUVMin = pAux->m_sUVDomain.GetMin();
//        SmPoint2d sUVMax = pAux->m_sUVDomain.GetMax();
//      
//        // build split cubicBezier child surfaces 
//        if (eSubdivideDirection == SM_SP_U) 
//          {
//            pAux->m_eSplitDir = SM_SP_U;
//            SER(pNodeSurface->ComputeCubicBezierSplit(pAux->m_sUVDomain,SM_SP_U,crOrigBez,
//                pAux1->m_sUVDomain,pAux2->m_sUVDomain,crBezier1,crBezier2));
//          }
//        else 
//          {
//            pAux->m_eSplitDir = SM_SP_V;
//            SER(pNodeSurface->ComputeCubicBezierSplit(pAux->m_sUVDomain,SM_SP_V,crOrigBez,
//                pAux1->m_sUVDomain,pAux2->m_sUVDomain,crBezier1,crBezier2));
//          }
//      
//        // set children bounding boxes
//        crBezier1.CalculateBoundingBox(&pNode1->m_sBBox, NULL, NULL) ;
//        crBezier2.CalculateBoundingBox(&pNode2->m_sBBox, NULL, NULL) ;
//      
//      #ifdef SM_DEBUG_CODE
//      SmBoolean bDebugMe2 = FALSE;
//        if (bDebugMe2) {
//            smgfx_Erase();
//            sm_GraphicsLoop();
//            smgfx_SetColor(1,0,0);
//            crBezier1.Draw();
//            sm_GraphicsLoop();
//            smgfx_SetColor(0,0,1);
//            crBezier2.Draw();
//            sm_GraphicsLoop();
//            smgfx_SetColor(0,0,0);
//            m_cpSurface->DrawUV(5,5);
//            sm_GraphicsLoop();
//        }
//      #endif
//      
//        return SM_SUCCESS;
//      
//      } // end SmSurfaceCache::SubdivideNodeFast


/*******************************************************************//**
PURPOSE: Compute the approximate chord heights and turning angles in the U and V direction. 

NOTES: Note that the chord height is only relative to an
    individual cross section of the control net.  An estimate of 
    the actual chord height can be obtained by adding these two
    chord heights.
***********************************************************************/
SmStatus SmSurfaceCache::ComputeNetConstants
  (const SmSurface  * pSurface,        // in : optional surface expected to be nonNULL for non SmBSplineSurface types
   const SmExtent2d * pUVDomain,       // in : only used when pSurface != NULL                                       
   const gw_SURFACE * cpSur,           // in : Shape being tested                                                    
   SmZoneTol3d      & rZoneTol3d,      // in :
   double           * pdUChordHeight,  // out: max Udir controlPoint dist from polygon BasePlane dist, NULL to ignore
   double           * pdVChordHeight,  // out: max Vdir controlPoint dist from polygon BasePlane dist, NULL to ignore
   double           * pdUAngleDeg,     // out: max Udir polygon endTangent angle, NULL to ignore                     
   double           * pdVAngleDeg,     // out: max Vdir polygon endTangent angle, NULL to ignore                     
   double           * pdUVChordHeight) // out: max UVdir diagonal controlPoint ist from polygon BasePlane dist, NULL to ignore
{
  SER(sm_ComputeNetConstants(pSurface,
                             pUVDomain,
                             cpSur,
                             rZoneTol3d,
                             pdUChordHeight,
                             pdVChordHeight,
                             pdUAngleDeg,
                             pdVAngleDeg,
                             pdUVChordHeight));
  return SM_SUCCESS;

} // end SmSurfaceCache::ComputeNetConstants

/*******************************************************************//**
PURPOSE: Compute the approximate chord heights and turning angles in the U and V direction. 

NOTES: Note that the chord height is only relative to an
    individual cross section of the control net.  An estimate of 
    the actual chord height can be obtained by adding these two
    chord heights.
***********************************************************************/

//  // Check the subdivision direction - by looking ahead and seeing how the children pass the subdivision tests
//  if(bSubdivide == TRUE)
//    {
//      double dAreaSplitU = LookAhead(SM_SP_U, lMoveCnt) ;
//      double dAreaSplitV = LookAhead(SM_SP_V, lMoveCnt) ;
//      eSubdivideDirection = (dAreaSplitU > dAreaSplitV) ? SM_SP_U : SM_SP_V ;
//    } // end need to check subdivision
       
SmStatus SmSurfaceCache::LookAhead
 (SmExtent2d        sUVDomain,          // in : sub domain for this look 
  ULONG             lMoveCnt,           // in : number of moves to look ahead 
  SmSurfParamType & eBestSplitDir,      // out: oneof of: SM_SP_U, SM_SP_V
  double          & dBestArea)          // out:
{
  if(lMoveCnt == 0) 
    {
      SmBoolean       bNeedsSubdivision ;
      SmSurfParamType eSubdivisionDirection ;
      TestAgainstTolerances(NULL, bNeedsSubdivision, eSubdivisionDirection) ;
      eBestSplitDir = eSubdivisionDirection ;
      dBestArea     = bNeedsSubdivision ? 0.0 : 100.0 ; 
    }

  if(lMoveCnt > 0)
    {
      SmPoint2d sMin =  sUVDomain.GetMin() ;
      SmPoint2d sMax =  sUVDomain.GetMax() ;
      SmPoint2d sMid = (sMin + sMax) / 2.0 ;
      double dAreaLeft, dAreaRight, dAreaBot, dAreaTop ;
      SmExtent2d sUVDomainLeft (sMin.x, sMin.y, sMid.x, sMax.y) ; 
      SmExtent2d sUVDomainRight(sMid.x, sMin.y, sMax.x, sMax.y) ; 
      SmExtent2d sUVDomainBot  (sMin.x, sMin.y, sMax.x, sMid.y) ; 
      SmExtent2d sUVDomainTop  (sMin.x, sMid.y, sMax.x, sMax.y) ;
      
      LookAhead(sUVDomainLeft , lMoveCnt-1, eBestSplitDir, dAreaLeft ) ;
      LookAhead(sUVDomainRight, lMoveCnt-1, eBestSplitDir, dAreaRight) ;
      LookAhead(sUVDomainBot  , lMoveCnt-1, eBestSplitDir, dAreaBot  ) ;
      LookAhead(sUVDomainTop  , lMoveCnt-1, eBestSplitDir, dAreaTop  ) ;

      double dAreaUSplit = (dAreaLeft + dAreaRight)/2.0 ;
      double dAreaVSplit = (dAreaBot  + dAreaTop)/2.0 ;

      if(dAreaUSplit > dAreaVSplit) { eBestSplitDir = SM_SP_U ;
                                      dBestArea     = dAreaUSplit ; 
                                    }
      else                          { eBestSplitDir = SM_SP_V ;
                                      dBestArea     = dAreaVSplit ; 
                                    }
    }
  
  return(SM_SUCCESS) ; 

} // end SmSurfaceChache::LookAhead  

/*******************************************************************//**
PURPOSE: Test the current tree node against the tolerances.

NOTES: 
***********************************************************************/
SmStatus SmSurfaceCache::TestAgainstTolerances
  (SmTreeNode      * pNode,                   // in : node to test                             
   SmBoolean       & rbNeedsSubdivision,      // out: TRUE = patch fails some tessellation test
   SmSurfParamType & reSubdivisionDirection)  // out: suggested direction to split bad elements
                                              //      oneof: SM_SP_U: Child1 = left, Child2 = right                         
                                              //             SM_SP_V: Child1 = bot,  Child2 = top                           
{
  // init output
  rbNeedsSubdivision     = FALSE;
  reSubdivisionDirection = SM_SP_U;

#ifdef SM_SBDV_LOG

// GWC_NOTE SET_NEXT_LINE_TO_TRUE_TO_LOG_SUBDIVISION GWC_LINE ;
static constexpr SmBoolean bLogMe = FALSE ;
  TCHAR sBuff[SM_TBLOCK_SIZE];
  if(bLogMe)
    {
      smos_sprintf(sBuff, _T("\n SmSurfaceCache::TestAgainstTolerances pNode[0x%p]: "), pNode) ;
      smos_WriteBuffer(sBuff);
    }
#endif // SM_SBDV_LOG

  // check state - nodes are properly labeled and have not yet been split
  if(   pNode->m_eAuxDataType != SM_AD_BEZIER_SURFACE
     && pNode->m_eAuxDataType != SM_AD_AUX_DATA) 
    { SER(SM_ERR); }
  
  if (pNode->m_pChild1 != NULL) 
    { SER(SM_ERR); }

  // locals
  SmSurfaceCache   *pSurfaceCache       = this;
  SmBezierAux2d    *pAux                = (SmBezierAux2d*)pNode->m_pData;
  SmPseudoBox       sPSBox, *pPSBox     = &sPSBox;
  gw_SURFACE       *pSur                = NULL;
  double            dUChordHeight, dVChordHeight, dUVChordHeight;
  double            dUAngDeg,      dVAngDeg;
  SmBoolean         bForceAngleTest ;
  SmBoolean         bSubdivide          = FALSE;
  SmSurfParamType   eSubdivideDirection = SM_SP_U;

  // local tolerances
  double            dChordHeightSquared =   pSurfaceCache->m_dChordHeightTolerance 
                                          * pSurfaceCache->m_dChordHeightTolerance;
  double            dAngTolDeg          =   pSurfaceCache->m_dAngTolRad * 180 / SM_PI;
  SmZoneTol3d       sZoneTol3d          =   SmTol::GetZoneTol3d( m_cpSurface );

  // local tolerance state - a zero ChordHeight or Angle Tol value means to ignore the test
  if(dChordHeightSquared < SM_EFF_ZERO) { pAux->m_bChordHeightSatisfied = TRUE; }
  if(dAngTolDeg          < SM_EFF_ZERO) { pAux->m_bAngleTolSatisfied    = TRUE; }
  SmBoolean bBothTolerancesSatisfied =     pAux->m_bChordHeightSatisfied
                                        && pAux->m_bAngleTolSatisfied ;
  
  // Get a pseudo box for this patche's uv-domain
  //    When node is from SmTessSrfCache::BuildTree for a BSplineSurface 
  //                 through a BuildTreeWithSubdivision() call, it has
  //                 an m_pData object WITHOUT a bezier gw_SURFACE patch
  //                 and an m_pData object with an mBA_pSurface value
  if (pNode->m_eAuxDataType == SM_AD_AUX_DATA) 
    {
      bForceAngleTest      = TRUE;
      pPSBox               = &sPSBox;
      SM_ASSERT(   pAux->mBA_pSurface   
                && pAux->mBA_pSurface->IsKindOf(SmBSplineSurface_TYPE)) ;
      pSur                 = ((SmBSplineSurface *)pAux->mBA_pSurface)->GetGwNurbPointer();

      if (!bBothTolerancesSatisfied) 
        {
          SER(pAux->mBA_pSurface->CalculateBoundingBox(pAux->m_sUVDomain,
                                                       &pNode->m_sBBox,
                                                       &sPSBox));
          SM_ASSERT_MSG(   pNode->m_pParent == NULL
                        || pNode->m_sBBox.IsContainedBy(pNode->m_pParent->m_sBBox, SM_EFF_ZERO_SQRT),_T("pChild BBox not contained in parent BBox")) ;

        }
    }
  else // node is from SmSurfaceCache::BuildTree for any kind of surface
       //         through a BuildTreeBase() call it has
       //         an m_pData object WITH a bezier gw_SURFACE patch 
       //         and an m_pData object WITHOUT an mBA_pSurface value.
    {
      SM_ASSERT(pNode->m_eAuxDataType == SM_AD_BEZIER_SURFACE)
      SmBezierPatch *pBezPatch = (SmBezierPatch*)pNode->m_pData;
      
      bForceAngleTest          = FALSE;
      pPSBox                   = &pBezPatch->m_sPseudoBox;
      pSur                     = pBezPatch->GetBezierPtr();
    }

  // When the surface is not a SmBSplineSurface (it came from SmSurfaceCache::BuildTree)
  //  - Get a Surface pointer and a subdomain.
  //  These values force sm_ComputeNetConstants() to generate
  //    approximate ChordHeight and Turning Angle values by sampling 
  //    the surface rather than just using the control net.
  const SmSurface  *pSurToTest = NULL;
  const SmExtent2d *pUVDomain  = NULL;
  if (!m_cpSurface->IsKindOf(SmBSplineSurface_TYPE)) 
    {
      pSurToTest = m_cpSurface;
      pUVDomain  = &pAux->m_sUVDomain;
    }

  // Note, when checking numbers, be careful about '<' vs. '<=', etc.:
  // it is possible for deviations to be exactly equal to the tolerances,
  // and if we're not completely consistent, it can cause problems. [B92]
  // Conventions:
  // - if tol == SM_EFF_ZERO exactly, we say it IS specified.
  // - if deviation == tol   exactly, we say it IS satisfied.

  // When asked, check chordheight and angtol properties and set bSubdivide as needed.
  //  note: Always check both individually, otherwise m_bSwitchToFastSubdivision
  //        will never be set to True, which makes for inconsistencies
  //        in how the tessellation parameters work together.
  //

  // chordheight
  if (dChordHeightSquared >= SM_EFF_ZERO) 
    {
      SmExtent1d sIvl1, sIvl2, sIvl3;
      pPSBox->GetIntervals(sIvl1,sIvl2,sIvl3);
      double dHeight = sIvl3.GetLength();
      if (dHeight*dHeight <= dChordHeightSquared) 
        { 
          pAux->m_bChordHeightSatisfied = TRUE ;
#ifdef SM_SBDV_LOG
          if(bLogMe)
            {
              smos_sprintf(sBuff, _T("OK ChordHeight3d[%16.16lf < %16.16lf], "), 
                                dHeight, 
                                pSurfaceCache->m_dChordHeightTolerance) ;
              smos_WriteBuffer(sBuff);
            }
#endif // SM_SBDV_LOG
           
        }
      else
        {
          bSubdivide = TRUE;

          // note: pSurToTest is only notNULL for non SmBSPlineSurface objects
          SER(sm_ComputeNetConstants(pSurToTest,        // in : notNULL = test surface values to compute outputs
                                     pUVDomain,         // in : only used when pSurToTest != NULL, target domain
                                     pSur,              // in : Always used - target shape to test
                                                        //      when pNode->m_eAuxDataType == SM_AD_AUX_DATA
                                                        //        pSur = SmBezierAux2d mBA_pSurface->gw_SURFACE
                                                        //      when pNode->m_eAuxDataType == SM_AD_BEZIER_SURFACE
                                                        //        pSur = SmBezierPatch gw_SURFACE
                                     sZoneTol3d,
                                     &dUChordHeight,    // out: max Udir controlPoint dist from polygon BasePlane dist, NULL to ignore 
                                     &dVChordHeight,    // out: max Vdir controlPoint dist from polygon BasePlane dist, NULL to ignore 
                                     NULL,              // out: max Udir polygon endTangent angle, NULL to ignore 
                                     NULL,              // out: max Vdir polygon endTangent angle, NULL to ignore
                                     &dUVChordHeight)); // out: max UV controlPoint dist from polygon BasePlane dist, NULL to ignore           
          // If U- and V- chord heights are within tol but the diagonal is not,
          // then figure which u/v direction to split by estimating which direction
          // is larger.  Get that estimate by multiplying the derivative magnitudes
          // at the center of the patch by the parameter ranges.
          if(   dUChordHeight  <= pSurfaceCache->m_dChordHeightTolerance 
             && dVChordHeight  <= pSurfaceCache->m_dChordHeightTolerance
             && dUVChordHeight >  pSurfaceCache->m_dChordHeightTolerance)
            {
              // evaluate first derivatives at center of patch
              double us, ue, vs, ve ;
              SmVector3d aDerivs[4] ; // 1d organized: [D, Dv, Du, Duv]
              NL_POINT *SD[2] ;
              SD[0] = (NL_POINT *) &aDerivs[0] ;  // NL_POINT and SmVector3d being just 3 doubles.
              SD[1] = (NL_POINT *) &aDerivs[2] ;  //   note: that will no longer be TRUE if SmVector3d gets a virtual function.
              N_SrfGetParameterBounds(pSur, &us, &ue, &vs, &ve) ;
              N_SrfDerivs(pSur, (us+ue)/2.0, (vs+ve)/2.0, NL_LEFT, NL_LEFT, TRUE, 1, 1, SD) ;
                  
              // split the longer direction
              if ( aDerivs[2].LengthSquared()*(ue-us) < aDerivs[1].LengthSquared()*(ve-vs) )
                { eSubdivideDirection = SM_SP_V; }
            }

          // Otherwise split the direction with the larger directional chordheight error.
          //   Note, we know we're splitting anyway, so don't check this
          //   when deciding whether to switch split directions. [B237]
          //   (Note also that the decision to split, just above, was made
          //   using the pseudobox, and now we're checking net constants.)
          else
            { 
              if ( TRUE  // (   dUChordHeight >= pSurfaceCache->m_dChordHeightTolerance 
                         //  || dVChordHeight >= pSurfaceCache->m_dChordHeightTolerance )
                  && dVChordHeight > dUChordHeight) 
                { 
                  eSubdivideDirection = SM_SP_V;
                }
            }
#ifdef SM_SBDV_LOG
      if(bLogMe)
        {
          smos_sprintf(sBuff, _T("Subdivide[%s]: ChordHeight3d[%16.16lf > %16.16lf]"), 
                            eSubdivideDirection == SM_SP_U ? _T("SM_SP_U") : _T("SM_SP_V"),
                            dHeight, 
                            pSurfaceCache->m_dChordHeightTolerance) ;
          smos_WriteBuffer(sBuff);
        }
#endif // SM_SBDV_LOG

        }
    } // end ChordHeight

  // AngleTol
  // Init these so we can tell whether they get set here:
  dUAngDeg = dVAngDeg = -1.0;

  // when needed - set eSubdivideDirection, bSubdivide,
  //                   pAux->m_bAngleTolSatisfied, and m_bSwitchToFastSubdivision values
  if (   bForceAngleTest                        // only TRUE for nodes from SmTessSrfCache::BuildTree() for SmBSplineSurface objects
      || (   !bSubdivide                        // when bForceAngleTest == TRUE, set m_bSwitchToFastSubdivision value
          && dAngTolDeg >= SM_EFF_ZERO))        // when bSubdivide == TRUE, skip resetting eSubdivideDirection 
    {
      // note: pSurToTest is only notNULL for non SmBSPlineSurface objects
      SER(sm_ComputeNetConstants(pSurToTest,        // in : notNULL = test surface values to compute outputs
                                 pUVDomain,         // in : only used when pSurToTest != NULL, target domain
                                 pSur,              // in : Always used - target shape to test
                                                    //      when pNode->m_eAuxDataType == SM_AD_AUX_DATA
                                                    //        pSUr = SmBezierAux2d mBA_pSurface->gw_SURFACE
                                                    //      when pNode->m_eAuxDataType == SM_AD_BEZIER_SURFACE
                                                    //        pSur = SmBezierPatch gw_SURFACE
                                 sZoneTol3d,
                                 NULL,              // out: max Udir controlPoint dist from polygon BasePlane dist, NULL to ignore
                                 NULL,              // out: max Vdir controlPoint dist from polygon BasePlane dist, NULL to ignore
                                 &dUAngDeg,         // out: max Udir polygon endTangent angle, NULL to ignore 
                                 &dVAngDeg,         // out: max Vdir polygon endTangent angle, NULL to ignore
                                 NULL)) ;           // out: max UV controlPoint dist from polygon BasePlane dist, NULL to ignore  

      // Returning more than 1-1/2 full revolutions indicates that the
      // control net is not suitable for estimating the angles.
      // In that case, use surface evaluations directly. [B99]
      if ( dUAngDeg > 540 || dVAngDeg > 540 )
        {
          const SmSurface *pSurf = ( pSurToTest != NULL ) ? pSurToTest : m_cpSurface;

          SER(sm_ComputeSurfConstants(pSurf,       // in: always used.
                                      pAux->m_sUVDomain,
                                      NULL,        // out: U chord height
                                      NULL,        // out: V chord height
                                      &dUAngDeg, // out: U end tangent angle
                                      &dVAngDeg, // out: V end tangent angle
                                      NULL));      // out: UV diagonal chord height
        }

      // Optimization for SmTessSrfCache::BuildTree().  When node angle
      //    goes below m_dStartFastSubdivisionAngleDeg value AND m_eTessellationAlgorithm == SM_TA_FASTER_TESSELLATION
      //    Then switch to SubdivideByBlock() from SubdivideNode() in SubdivideToTolerances().
                                                                     
      // note: bForceAngleTest is only true for nodes from SmTessSrfCache::BuildTreeWithSubdivision()
      //                       when pAux->mBA_pSurface != NULL
      if(   bForceAngleTest
         && dUAngDeg < m_dStartFastSubdivisionAngleDeg
         && dVAngDeg < m_dStartFastSubdivisionAngleDeg) 
        {
          // Specify upcoming SubdivideToTolerances() behavior
          //   TRUE = when m_eTessellationAlgorithm == SM_TA_FASTER_TESSELLATION: call SubdivideByBlock()
          //          else                                                        call SubdivideNode()
          //   FALSE= call SubdivideNode()
          m_bSwitchToFastSubdivision = TRUE;
#ifdef SM_SBDV_LOG
          if(bLogMe)
            {
              smos_sprintf(sBuff, _T("Begin FastSubdivision (turning AngDeg small) [%16.16lf <= %16.16lf], "), 
                                smos_Max(dUAngDeg, dVAngDeg), 
                                m_dStartFastSubdivisionAngleDeg) ;
              smos_WriteBuffer(sBuff);
            }
#endif // SM_SBDV_LOG

        }

      // look for passing/failing patches
      if(   ( dAngTolDeg < SM_EFF_ZERO) // meaningless to test against 0.0
         || (    dUAngDeg <= dAngTolDeg 
              && dVAngDeg <= dAngTolDeg) )
        { 
          pAux->m_bAngleTolSatisfied = TRUE;

#ifdef SM_SBDV_LOG
          if(bLogMe)
            {
              smos_sprintf(sBuff, _T("OK AngTolDeg3d[%16.16lf <= %16.16lf], "), 
                                smos_Max(dUAngDeg, dVAngDeg), 
                                dAngTolDeg) ;
              smos_WriteBuffer(sBuff);
            }
#endif // SM_SBDV_LOG
           
        }
      else
        {
          // When direction is not yet picked - Split the direction with the larger angle.
          if(   !bSubdivide
             && dVAngDeg > dUAngDeg) 
            { eSubdivideDirection = SM_SP_V; }
          bSubdivide = TRUE;

#ifdef SM_SBDV_LOG
          if(bLogMe)
            {
              smos_sprintf(sBuff, _T("Subdivide[%s]: AngTolDeg3d[%16.16lf > %16.16lf]"), 
                                eSubdivideDirection == SM_SP_U ? _T("SM_SP_U") : _T("SM_SP_V"),
                                smos_Max(dUAngDeg, dVAngDeg), 
                                dAngTolDeg) ;
              smos_WriteBuffer(sBuff);
            }
#endif // SM_SBDV_LOG

        }
    } // end AngleTol

  // arrive here after setting
  //   bSubdivde,                   TRUE = needs more subdivision
  //   eSubdivideDirection,         SM_SP_V or SM_SP_U, specifies direction of split
  //   m_bSwitchToFastSubdivision,  TRUE = chordheight not satisfied but angTol is satisfied

  // when Node passes ChordHeight and AngleTol checks --
  // this is a flat patch (or, those two parameters weren't specified);
  // time for other tests.
  //   AspectRatio3D, 
  //   MaximumSideLength3D, 
  //   MinimumSideLength3D

  // node property locals
  SmPoint2d  sAspectRatio, sMaxSideLength3D, sMinSideLength3D;

  // when further testing is required - compute node properties
  if(   (   !bSubdivide
         && (   m_dAspectRatio3D       > 0.0 
             || m_dMaximumSideLength3D > 0.0))  // may need to add a subdivision
     || (   bSubdivide
         && m_dMinimumSideLength3D > 0.0))      // may need to stop a subdivsion
    {
      // locals
      SmExtent2d sDomain;
      SmPoint3d  sU0V0, sU1V0, sU1V1, sU0V1;

      // compute dUAngDeg and dVAngDeg to see if patch is flat
      if(   dUAngDeg < 0.0 
         || dVAngDeg < 0.0 )
        {
          SER(sm_ComputeNetConstants(pSurToTest,  // in : optional surface expected to be nonNULL for non SmBSplineSurface types
                                     pUVDomain,   // in : only used when pSurface != NULL
                                     pSur,        // in : Shape being tested - always used
                                     sZoneTol3d,  // in :
                                     NULL,        // out: max Udir controlPoint dist from polygon BasePlane dist, NULL to ignore
                                     NULL,        // out: max Vdir controlPoint dist from polygon BasePlane dist, NULL to ignore
                                     &dUAngDeg,   // out: max Udir polygon endTangent angle, NULL to ignore
                                     &dVAngDeg,   // out: max Vdir polygon endTangent angle, NULL to ignore
                                     NULL)) ;     // out: max UV controlPoint dist from polygon BasePlane dist, NULL to ignore
            
        } // end need to compute dUAngDeg and dVAngDeg check

      // Compute patch AspectRatio, MaxSideLength, MinSideLength

      // when patch is Flat enough, compute from surface corner points - cheap
      if(   dUAngDeg <= dAngTolDeg
         && dVAngDeg <= dAngTolDeg)
        {
          // Get Surface 3d corners
          SER(GetCorners(pSur,pAux->m_sUVDomain,sU0V0,sU1V0,sU1V1,sU0V1,sDomain));

          // compute from 3d corner set - cheap
          SER(ComputeSizeTolerances(sU0V0,              // in : 3d corner of Surface(u0, v0) of target quad
                                    sU1V0,              // in : 3d corner of Surface(u1, v0) of target quad
                                    sU1V1,              // in : 3d corner of Surface(u1, v1) of target quad
                                    sU0V1,              // in : 3d corner of Surface(u0, v1) of target quad
                                    sAspectRatio,       // out: [dMaxU/dMaxV, dMaxV/dMaxU] in 3d, except Max < dScaledZero, then 0.5
                                    sMaxSideLength3D,   // out: [dMaxU, dMaxV] in 3d,
                                    sMinSideLength3D)); // out: [dMinU, dMinV] in 3d, except Min < dScaledZero, then Max
        } // end flat branch
      else // when patch is not flat enough, compute from surface iso-curves - expensive.
        {
          // compute from sample surface IsoCurves - expensive
          SER( ComputeSizeTolerances(m_cpSurface,        // in : Target Surface
                                      pAux->m_sUVDomain,  // in : Target Surface sub UVDomain
                                      sAspectRatio,       // out: [dMaxU/dMaxV, dMaxV/dMaxU] in 3d, except Max < dScaledZero, then 0.5
                                      sMaxSideLength3D,   // out: [dMaxU, dMaxV] in 3d 
                                      sMinSideLength3D)); // out: [dMinU, dMinV] in 3d, except Min < dScaledZero, then Max
        } // end not flat branch
    } // end need to compute Node properties check

  // when asked - force division until MaxSideLengths < limit
  if(   !bSubdivide
     && m_dMaximumSideLength3D > 0.0
     && (sMaxSideLength3D.Length() > m_dMaximumSideLength3D ) )
    {
      bSubdivide = TRUE;

      // always divide biggest dimension
      eSubdivideDirection =   (sMaxSideLength3D.x > sMaxSideLength3D.y)
                            ? SM_SP_U
                            : SM_SP_V ; 

#ifdef SM_SBDV_LOG
      if(bLogMe)
        {
          smos_sprintf(sBuff, _T("Subdivide[%s]: MaxSideLength3d[%16.16lf > %16.16lf]"), 
                            eSubdivideDirection == SM_SP_U ? _T("SM_SP_U") : _T("SM_SP_V"),
                            sMaxSideLength3D.Length(), 
                            m_dMaximumSideLength3D) ;
          smos_WriteBuffer(sBuff);
        }
#endif // SM_SBDV_LOG

    } // end MaxSideLengths limit check

  // when asked - force division until AspectRatio < limit
  if(   !bSubdivide
      && m_dAspectRatio3D > 0.0 
      && (   sAspectRatio.x > m_dAspectRatio3D 
          || sAspectRatio.y > m_dAspectRatio3D)) 
    {
      bSubdivide = TRUE;

      // always divide biggest dimension
      eSubdivideDirection =   (sMaxSideLength3D.x > sMaxSideLength3D.y)
                            ? SM_SP_U
                            : SM_SP_V ; 

#ifdef SM_SBDV_LOG
      if(bLogMe)
        {
          smos_sprintf(sBuff, _T("Subdivide[%s]: AspectRatio3d[%16.16lf > %16.16lf]"), 
                            eSubdivideDirection == SM_SP_U ? _T("SM_SP_U") : _T("SM_SP_V"),
                            smos_Max(sAspectRatio.x, sAspectRatio.y), 
                            m_dAspectRatio3D) ;
          smos_WriteBuffer(sBuff);
        }
#endif // SM_SBDV_LOG

    } // end AspectRatio limit check

  // when asked - stop division when MinSideLength < limit
  if(   bSubdivide
     && m_dMinimumSideLength3D > 0.0
     && sMinSideLength3D.x <= m_dMinimumSideLength3D 
     && sMinSideLength3D.y <= m_dMinimumSideLength3D) 
    {
      bSubdivide = FALSE ;

#ifdef SM_SBDV_LOG
      if(bLogMe)
        {
          smos_sprintf(sBuff, _T("\n    SubdivideStopped: MinSideLength3d[%16.16lf <= %16.16lf]"), 
                            smos_Max(sMaxSideLength3D.x, sMinSideLength3D.y), 
                            m_dMinimumSideLength3D) ;
          smos_WriteBuffer(sBuff);
        }
#endif // SM_SBDV_LOG

    }

  // no more work - patch passes all subdivision tests
  if (!bSubdivide) 
    {
#ifdef SM_SBDV_LOG
      if(bLogMe)
        {
          smos_WriteBuffer(_T("OK Node - no Subdivision"));
        }
#endif // SM_SBDV_LOG
      return SM_SUCCESS;  // No further subdivision - passed all tests
    }

  // Perform a test here to make sure that we eventually stop subdivision
  // For now don't let either node direction get less than 1/1000th of
  // size of original UV domain
  SmVector2d sUVSize   = m_sUVDomain.GetSize();
  SmVector2d sNodeSize = pAux->m_sUVDomain.GetSize();
  double     dMinSLR   =  (m_dMinimumSideLengthRatioUV != 0.0) 
                         ? m_dMinimumSideLengthRatioUV 
                         : 0.001;

  // USide or VSide too small When being split
  if(   (eSubdivideDirection == SM_SP_U && sNodeSize.x < sUVSize.x * dMinSLR)
     || (eSubdivideDirection == SM_SP_V && sNodeSize.y < sUVSize.y * dMinSLR))
    {
      // if USide or VSide is big enough - split it, otherwise stop splitting
      if     (   eSubdivideDirection == SM_SP_U 
              && sNodeSize.y > sUVSize.y / 100.0) { eSubdivideDirection = SM_SP_V; 
#ifdef SM_SBDV_LOG
                                                    if(bLogMe)
                                                      {
                                                        smos_sprintf(sBuff, _T("\n    SubdivideChanged[%s]: MinUSideUV[%16.16lf < %16.16lf]"), 
                                                                          _T("SM_SP_V"),
                                                                          sNodeSize.x, 
                                                                          sUVSize.x * dMinSLR) ;
                                                        smos_WriteBuffer(sBuff);
                                                      }
#endif // SM_SBDV_LOG
                                                  }
      else if(   eSubdivideDirection == SM_SP_V 
              && sNodeSize.x > sUVSize.x / 100.0) { eSubdivideDirection = SM_SP_U;  
#ifdef SM_SBDV_LOG
                                                    if(bLogMe)
                                                      {
                                                        smos_sprintf(sBuff, _T("\n    SubdivideChanged[%s]: MinVSideUV[%16.16lf < %16.16lf]"), 
                                                                          _T("SM_SP_U"),
                                                                          sNodeSize.y, 
                                                                          sUVSize.y * dMinSLR) ;
                                                        smos_WriteBuffer(sBuff);
                                                      }
#endif // SM_SBDV_LOG
                                                  }
      else                                        { rbNeedsSubdivision = FALSE;
#ifdef SM_SBDV_LOG                                
                                                    if(bLogMe)
                                                      {
                                                        smos_sprintf(sBuff, _T("\n    SubdivideStopped: MinNodeSizeUV[%16.16lf < %16.16lf] and [%16.16lf < %16.16lf]"), 
                                                                          sNodeSize.x, sUVSize.x * dMinSLR,
                                                                          sNodeSize.y, sUVSize.y * dMinSLR) ;
                                                        smos_WriteBuffer(sBuff);
                                                      }
#endif // SM_SBDV_LOG                             
                                                    return SM_SUCCESS;
                                                  }
    } // end USide or VSide too small When being split check                                            
                                                 
  // set output for split
  rbNeedsSubdivision     = TRUE;
  reSubdivisionDirection = eSubdivideDirection;

  // all done
  return SM_SUCCESS;

} // end SmSurfaceCache::TestAgainstTolerances

/*******************************************************************//**
PURPOSE: Static helper routine: reduce the size of an array by half
            by removing every second element.

NOTES: Always retains the first and last elements.
***********************************************************************/
static SmStatus sm_CullArray( SmTArray<double> & sArr )
{
  ULONG ii, jj, lSize = sArr.GetSize();
  for ( ii = 2, jj = 1; ii < lSize; ii += 2, jj += 1 )
  {
    sArr[jj] = sArr[ii];
  }
  // Make sure the last element is retained.
  if ( ii == lSize )
    { sArr[jj] = sArr[ lSize-1 ]; jj++; }

  sArr.SetSize( jj );

  return SM_SUCCESS;

} // end sm_CullArray

/*******************************************************************//**
PURPOSE: Subdivide a surface by picking tolerance satisfying
            iso-curves partitions.

NOTES: 
   1. Expects the input TreeNode->m_eAuxDataType == SM_AD_AUX_DATA and builds
      all descendants as SM_AD_AUX_DATA (without bezier patches) as well.
   2. preserves the following subdivision rule for every parent/child relation
       for pParentNode->m_pData->m_eSplitDir = oneof SM_SP_U, SM_SP_V, where
              SM_SP_U: make new Child1 = left
                                Child2 = right
              SM_SP_V: make new Child1 = bot
                                Child2 = top
***********************************************************************/
SmStatus SmSurfaceCache::SubdivideByBlock(SmTree* pTree, // in : Tree under construction
                                          SmTreeNode* pNode) // in : target node
{
    // check state
    SM_ASSERT(pNode->m_eAuxDataType == SM_AD_AUX_DATA);

    // locals
    SmBezierAux2d* pAux = (SmBezierAux2d*)pNode->m_pData;
    const SmSurface* pSurface = pAux->mBA_pSurface;
    NER(pSurface);
    SmExtent2d sUVDomain = pSurface->GetNaturalUVDomain();

    SM_ASSERT(pSurface->IsKindOf(SmBSplineSurface_TYPE));

#ifdef SM_DEBUG_CODE
    SmBoolean bDebugMe = FALSE;
    if (bDebugMe)
    {
        smgfx_Erase();
        smgfx_SetLook(1, 2, 0, 1, 1);
        pSurface->DrawUV(5, 5);
        sm_GraphicsLoop();
        sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

    // Find acceptable U and V split points by tessellating sample isoParameter Curves
    double sDData[256], sDData2[256];
    SmTArray<double> sUSplits(256, sDData);
    SmTArray<double> sVSplits(256, sDData2);

    // remember need to check diagonal when all natural isoParam Curves are linear
    // i.e. non-planar surfaces with linear edges.
    SmBoolean bCheckDiagonals = FALSE;

    // All derived types will have curved isoparams in at least one direction,
    // except for planes and possibly extruded surfaces.
    // And we don't have to check diagonals for a true plane.
    SM_TYPE eSurfaceType = pSurface->GetType();
    if (eSurfaceType == SmBSplineSurface_TYPE || eSurfaceType == SmSurfOfExtrusion_TYPE)
    {
        double dTol = m_dChordHeightTolerance > 0.0 ? m_dChordHeightTolerance / 2.0 : SM_EFF_ZERO_SQRT;
        SmIsoCurve sU0(*pSurface, SM_SP_U, sUVDomain.GetMin().x, TRUE);

        if (sU0.IsLinear(dTol))
        {
            SmIsoCurve sV0(*pSurface, SM_SP_V, sUVDomain.GetMin().y, TRUE);

            if (sV0.IsLinear(dTol))
            {
                SmIsoCurve sU1(*pSurface, SM_SP_U, sUVDomain.GetMax().x, TRUE);
                if (sU1.IsLinear(dTol))
                {
                    SmIsoCurve sV1(*pSurface, SM_SP_V, sUVDomain.GetMax().y, TRUE);
                    if (sV1.IsLinear(dTol))
                    {
                        // remember when all natural boundary isoParam curves are linear
                        bCheckDiagonals = TRUE;
                    } // end sV1 IsLinear check
                } // end sU1 IsLinear check
            } // sV0.IsLinear check
        } // end sU0 isLinear check
    } // end bCheckDiagonal check

    // UDir tessellation points
    SER(pSurface->FindTessellationSplits(SM_SP_U, // in : oneof: SM_SP_U, SM_SP_V
                                         sUVDomain, // in : surface domain
                                         this->m_dChordHeightTolerance, // in : max height/length allowed ratio,
                                                                        // 0=ignore
                                         this->m_dAngTolRad * 180.0 / SM_PI, // in : max angle between tessellation pt
                                                                             // tangents, 0=ignore
                                         this->m_dMaximumSideLength3D, // in : max length between tessellation pts,
                                                                       // 0=ignore
                                         sVSplits, // out: array of tessellation pts
                                                   //      (including end-pts, so size >= 2 always)
                                         bCheckDiagonals)); // in : TRUE = when U and V dirs are linear tesselate
                                                            // diagonal isoparameter curves
                                                            //      FALSE=don't, default:[FALSE]

    // VDir tessellation points
    SER(pSurface->FindTessellationSplits(SM_SP_V, // in : oneof: SM_SP_U, SM_SP_V
                                         sUVDomain, // in : surface domain
                                         this->m_dChordHeightTolerance, // in : max height/length allowed ratio,
                                                                        // 0=ignore
                                         this->m_dAngTolRad * 180 / SM_PI, // in : max angle between tessellation pt
                                                                           // tangents, 0=ignore
                                         this->m_dMaximumSideLength3D, // in : max length between tessellation pts,
                                                                       // 0=ignore
                                         sUSplits, // out: array of tessellation pts
                                                   //      (including end-pts, so size >= 2 always)
                                         bCheckDiagonals)); // in : TRUE = when U and V dirs are linear tesselate
                                                            // diagonal isoparameter curves
                                                            //      FALSE=don't, default:[FALSE]

    // Limit the total number of splits.
    // If inappropriate tessellation parameters are used,
    // this total can be large enough to cause memory errors.
    static constexpr ULONG slMaxTotalFacets = 100000;
    ULONG lNumSplitsU = sUSplits.GetSize();
    ULONG lNumSplitsV = sVSplits.GetSize();
    ULONG lTotal = lNumSplitsU * lNumSplitsV;

    // no work - no splits
    if (lNumSplitsU <= 2 && lNumSplitsV <= 2)
    {
        return (SM_SUCCESS);
    }

    // cull split points until total facet count is small enough to proceed
    while (lTotal > slMaxTotalFacets)
    {
        if (lNumSplitsU > 20)
        {
            sm_CullArray(sUSplits);
            lNumSplitsU = sUSplits.GetSize();
        }
        if (lNumSplitsV > 20)
        {
            sm_CullArray(sVSplits);
            lNumSplitsV = sVSplits.GetSize();
        }
        lTotal = lNumSplitsU * lNumSplitsV;

    } // end while culling points to reduce total facet count

    // create (sUSplits.GetSize()-1) * (sVSplits.GetSize()-1) base nodes
    //        which share a total of (sUSplits.GetSize()) * (sVSplits.GetSize()) TreeVertices
    //        four of which already exist.
    //  note: sUSplits and sVSplits include the end UV Values which don't need to be split
    //        so sUSplits.GetSize() == 3 ==> split once.
    SmTArray<SmTreeNode*> sBaseNodes;

    // TreeVertex Index Maps - needed to create SmTree::m_sTreeVertices entries when child nodes are made
    SmTArray<ULONG> sTreeVertexMap, sTreeVertexPre, sTreeVertexPost;

    // GWCTreeVertexTemp
    //  // Build the TreeVertex Index, Next, and Last maps needed to assign TreeVertex indices to every child node about
    //  to be built - increment m_sTreeVertices as needed pTree->BuildTreeVertexMaps
    //    (pNode,              // in : Parent Node being split before TreeVertices entries are updated or NULL for none
    //     sUSplits,           // in : rUSplits[i] = ith Parent Node Split Point with bndrys. Size of 2=no splits, 3=1
    //     split, . . . sVSplits,           // in : rVSplits[j] = jth Parent Node Split Point with bndrys. Size of 2=no
    //     splits, 3=1 split, . . . sTreeVertexMap,     // out: Index for every U/V split TreeVertex intersection
    //     sTreeVertexPre,     // out: closest TreeVertexIndex to This between This and Last U/V SplitPoints,
    //                         //      SM_BIG_ULONG = no TreeVertices between This and Last U/V SplitPoint
    //     sTreeVertexPost) ;  // out: closest TreeVertexIndex to This between This and Next U/V SplitPoints,
    //                         //      SM_BIG_ULONG = no TreeVertices between This and Next U/V SplitPoint

    // GWCTreeVertexTemp
    //   // The current node is about to become a parent node - internal nodes don't have TreeVertex loops
    //   pAux->m_lStartTreeVertexIndx = SM_BIG_ULONG ;

    // for every split - get and init new TreeNodes with new SmBezierAux2d m_pData Objects

{
  for (ULONG i=1; i<lNumSplitsU; i++) 
    {
      for (ULONG j=1; j<lNumSplitsV; j++) 
        {
          // locals
          SmExtent2d      sNodeUVDomain(sUSplits[i-1], sVSplits[j-1], sUSplits[i], sVSplits[j]) ;
          SmTreeNode    * pBaseNode = (SmTreeNode*)pTree->m_sNodeMgr.GetNewElement();
          SmBezierAux2d * pBaseAux  = (SmBezierAux2d*)this->m_sAuxMgr.GetNewElement();
          if (!pBaseAux) { SE(SM_ERR);
                           break;
                         }

// GWCTreeVertexTemp
//          // create/connect this node's TreeVertex Loop
//          pBaseAux->m_lStartTreeVertexIndx = m_pTree->SetTreeVertexLoop
//                                              (pBaseNode,           // in : TreeNode owner of this TreeVertex Loop
//                                               sNodeUVDomain,       // in : UVDomain for Node with this loop
//                                               i-1, j,              // in : Above Left Corner index[i,j] of new Node TreeVertex Loop
//                                               sUSplits, sVSplits,  // in : Total Number of U and V Splits in TreeVertex Array(ii,jj)
//                                               sTreeVertexMap,      // in : Array made by BuildTreeVertexMaps call
//                                               sTreeVertexPre,      // in : Array Made By BuildTreeVertexMaps call
//                                               sTreeVertexPost) ;   // in : Array Made By BuildTreeVertexMaps call

          // set this node's data
          sBaseNodes.Add(pBaseNode);

          pBaseNode->m_pTree            = pTree;
          pBaseNode->m_pChild1          = NULL;
          pBaseNode->m_pChild2          = NULL;
          pBaseNode->m_pParent          = NULL;
          // gwc: skipped pBaseNode->m_sBBox until later
          pBaseNode->m_eAuxDataType     = SM_AD_AUX_DATA;
          pBaseNode->m_pData            = pBaseAux;
          pBaseNode->m_eGeomType        = SM_NG_DEFAULT;

          pBaseAux->m_pOwningTreeNode       = pBaseNode;
          pBaseAux->m_eSplitDir             = SM_SP_U;
          pBaseAux->m_bIsMarked             = FALSE;
          pBaseAux->m_eNodeClass            = SM_NC_UNKNOWN;
          pBaseAux->m_pFace                 = NULL;
          pBaseAux->m_sUVDomain             = sNodeUVDomain ;
          pBaseAux->m_vVertexList.Init();
          pBaseAux->m_sEdgeuseList.Init();
          pBaseAux->m_sPolyEdgeList.Init();
          pBaseAux->mBA_pSurface            = (SmBSplineSurface*)pSurface;  // note the surface is not subdivided - its just the parent node's surface
          // gwc: skipped pBaseAux->m_sPolarBox until later
          pBaseAux->m_bChordHeightSatisfied = FALSE;
          pBaseAux->m_bAngleTolSatisfied    = FALSE;
          
          // for pBaseNode->m_sBBox and pBaseAux->m_sPolarBox
          SmBSplineSurface *pPatch = NULL ;
          ((SmBSplineSurface*)pSurface)->CopySubPatch(*GetContext(), sNodeUVDomain, pPatch) ;

          // If that fails, do what we can and continue.  [B429]
          if ( pPatch == NULL )
            {
          //    // Set Polar box to the guess.
          //    pBaseAux->m_sPolarBox = pAux->m_sPolarBox;

              continue;
            }

          SmObjDelete sClean(pPatch) ; 

          // set pBaseNode->m_sBBox and pBaseAux->m_sPolarBox
          pPatch->CalculateBoundingBox( pPatch->GetNaturalUVDomain(), // in : only whole surfaces are bounded (except for polarBoxes of analytic surfaces) 
                                       &pBaseNode->m_sBBox,           // out: Axis alligned box
                                        NULL,                         // out: Non-axis aligned box
                                        NULL,                         // out: Surface normal vector field bounding box
                                        NULL,                         // in : guess for PseudoBox basis vectors - used unless another better orientation is found
                                                                      //      NULL to ignore, default:[NULL]
                                        NULL) ;                       // in : guess for PolarBox basis vectors - used unless another better orientation is found
                                                                      //      NULL to ignore, default:[NULL]

          SM_ASSERT_MSG(pBaseNode->m_sBBox.IsContainedBy(pNode->m_sBBox, SM_EFF_ZERO_SQRT),_T("SmSurfaceCache::SubdivideByBlock - BlockChild BBox not contained in BlockRoot BBox")) ;

        } // end iter j = every internal VSplit
    } // end iter i = every internal USplit
}

// GWCTreeVertexTemp
//  #ifdef SM_DEBUG_CODE
//    if(bDebugAssert)
//      {
//        SmBoolean bRtn = m_pTree->AssertTreeVertices() ;
//        m_pTree->DumpTreeVertices() ;
//        SM_ASSERT_BREAK(bRtn) ;
//      }
//  #endif // SM_DEBUG_CODE
  
  // assemble the base nodes into a binary tree with a sequence of parent node generations
  ULONG lNumU = sUSplits.GetSize() - 1;
  ULONG lNumV = sVSplits.GetSize() - 1;

  SmTArray<SmTreeNode*> sNodes(sBaseNodes.GetSize());
  ULONG lErr = sNodes.Append(sBaseNodes);
  if ( lErr >= SM_BIG_ULONG )
    { SER( SM_ERR ); }  // Ran out of memory.  Don't want to crash.
  SmTArray<SmTreeNode*> sNodes2(sBaseNodes.GetSize());
  SmTArray<SmTreeNode*> *pCurr = &sNodes;
  SmTArray<SmTreeNode*> *pNext = &sNodes2;

  // assemble U and V splits until there is just one
  while (lNumU > 1 || lNumV > 1) 
    {
      SmSurfParamType sCombineDirection = (lNumV > lNumU) 
                                          ? SM_SP_V 
                                          : SM_SP_U;
      SmBoolean bLastTime = (lNumU + lNumV == 3) ? TRUE : FALSE;

      // Now we have to do something different depending upon which 
      // direction we are combining.  Note that the array contains
      // U ordered patches.  
      // Note that patches are ordered in such a way that the nodes with
      // a constant V are sequential.
      if (sCombineDirection == SM_SP_V) 
        {  // Combine along rows of V
          // This combination shrinks the rows
          for (ULONG ii=0; ii<lNumU; ii++) 
            {
              for (ULONG kk=1; kk<lNumV; kk=kk+2) 
                {
                  SmTreeNode *pNode1 = (SmTreeNode*)(*pCurr)[ii*lNumV+kk-1];
                  SmTreeNode *pNode2 = (SmTreeNode*)(*pCurr)[ii*lNumV+kk];
                  if (pNode1 == NULL || pNode2 == NULL)
                      return (SM_ERR);
                  SmTreeNode *pNewNode = NULL;
                  if (bLastTime) 
                    { 
                      pNewNode = pNode; 
                    }
                  else 
                    {                   
                      pNewNode = (SmTreeNode*)pTree->m_sNodeMgr.GetNewElement(); 
                      SmBezierAux2d *pBezAux  = (SmBezierAux2d*)m_sAuxMgr.GetNewElement();
                      if ( pNode1 == NULL || pNode2 == NULL  || pBezAux == NULL)
                      {
                          SE( pBezAux != NULL);
                          continue;
                      } 
                      SmBezierAux2d *pAux1 = (SmBezierAux2d*)pNode1->m_pData;
                      SmBezierAux2d *pAux2 = (SmBezierAux2d*)pNode2->m_pData;                 
                      pNewNode->m_pTree             = pTree;
                      pNewNode->m_pParent           = NULL;
                      pNewNode->m_eAuxDataType      = SM_AD_AUX_DATA;
                      pNewNode->m_pData             = pBezAux;
                      pNewNode->m_eGeomType         = SM_NG_DEFAULT;
                      
                      // pBezAux->m_eSplitDir             = SM_SP_V;
                      pBezAux->m_pOwningTreeNode       = pNewNode;
                      pBezAux->m_bIsMarked             = FALSE;
                      pBezAux->m_eNodeClass            = SM_NC_UNKNOWN;
                      pBezAux->m_pFace                 = NULL;
                      pAux1->m_sUVDomain.Union(pAux2->m_sUVDomain, pBezAux->m_sUVDomain);
                      pBezAux->m_vVertexList.Init();
                      pBezAux->m_sEdgeuseList.Init();
                      pBezAux->m_sPolyEdgeList.Init();
                      pBezAux->mBA_pSurface            = NULL;
                      pBezAux->m_bChordHeightSatisfied = FALSE;
                      pBezAux->m_bAngleTolSatisfied    = FALSE;

                      // combine child bounding boxes
                      pNode1->m_sBBox.Union(pNode2->m_sBBox, pNewNode->m_sBBox) ;
                      //pAux1->m_sPolarBox.Union(pAux2->m_sPolarBox, pBezAux->m_sPolarBox) ;

                      SM_ASSERT_MSG(pNode1->m_sBBox.IsContainedBy(pNewNode->m_sBBox, SM_EFF_ZERO_SQRT),_T("pChild1 BBox not contained in parent BBox")) ;
                      SM_ASSERT_MSG(pNode2->m_sBBox.IsContainedBy(pNewNode->m_sBBox, SM_EFF_ZERO_SQRT),_T("pChild2 BBox not contained in parent BBox")) ;
                    }
                    ((SmBezierAux2d*)pNewNode->m_pData)->m_eSplitDir = SM_SP_V;
                    pNext->Add(pNewNode);
                    pNewNode->m_pChild1 = pNode1;
                    pNewNode->m_pChild2 = pNode2;
                    pNode1->m_pParent = pNewNode;
                    pNode2->m_pParent = pNewNode;
                }
              if (lNumV % 2 == 1) 
                {
                  pNext->Add((*pCurr)[ii*lNumV+lNumV-1]);  // Add on last element if had odd number
                }
            }
          lNumV = (lNumV+1) / 2;  // Used up either half or one less columns 
        }
      else 
        { // Combine in V direction
          for (ULONG ii=1; ii<lNumU; ii=ii+2) 
            {
              for (ULONG kk=0; kk<lNumV; kk++) 
                {
                  SmTreeNode *pNode1 = (SmTreeNode*)(*pCurr)[(ii-1)*lNumV+kk];
                  SmTreeNode *pNode2 = (SmTreeNode*)(*pCurr)[ii*lNumV+kk];
                  if (pNode1 == NULL || pNode2 == NULL)
                      return (SM_ERR);

                  SmTreeNode *pNewNode = NULL;
                  if (bLastTime) 
                    { 
                      pNewNode = pNode; 
                    }
                  else 
                    {   
                      pNewNode = (SmTreeNode*)pTree->m_sNodeMgr.GetNewElement();
                      SmBezierAux2d *pBezAux = (SmBezierAux2d*)m_sAuxMgr.GetNewElement();
                      if ( pNode1 == NULL || pNode2 == NULL  || pBezAux == NULL)
                      {
                          SE( pBezAux != NULL);
                          continue;
                      }        
                      SmBezierAux2d *pAux1 = (SmBezierAux2d*)pNode1->m_pData;
                      SmBezierAux2d *pAux2 = (SmBezierAux2d*)pNode2->m_pData;
                      
                      pNewNode->m_pTree             = pTree;
                      pNewNode->m_pParent           = NULL;
                      pNewNode->m_eAuxDataType      = SM_AD_AUX_DATA;
                      pNewNode->m_pData             = pBezAux;
                      pNewNode->m_eGeomType         = SM_NG_DEFAULT;
                      
                      // pBezAux->m_eSplitDir             = SM_SP_U;
                      pBezAux->m_pOwningTreeNode       = pNewNode;
                      pBezAux->m_bIsMarked             = FALSE;
                      pBezAux->m_eNodeClass            = SM_NC_UNKNOWN;
                      pBezAux->m_pFace                 = NULL;
                      pAux1->m_sUVDomain.Union(pAux2->m_sUVDomain, pBezAux->m_sUVDomain);
                      pBezAux->m_vVertexList.Init();
                      pBezAux->m_sEdgeuseList.Init();
                      pBezAux->m_sPolyEdgeList.Init();
                      pBezAux->mBA_pSurface            = NULL;
                      pBezAux->m_bChordHeightSatisfied = FALSE;
                      pBezAux->m_bAngleTolSatisfied    = FALSE;
                      
                      // combine child bounding boxes
                      pNode1->m_sBBox.Union(pNode2->m_sBBox, pNewNode->m_sBBox) ;
                      //pAux1->m_sPolarBox.Union(pAux2->m_sPolarBox, pBezAux->m_sPolarBox) ;
                      
                      SM_ASSERT_MSG(pNode1->m_sBBox.IsContainedBy(pNewNode->m_sBBox, SM_EFF_ZERO_SQRT),_T("pChild1 BBox not contained in parent BBox")) ;
                      SM_ASSERT_MSG(pNode2->m_sBBox.IsContainedBy(pNewNode->m_sBBox, SM_EFF_ZERO_SQRT),_T("pChild2 BBox not contained in parent BBox")) ;
                    }
                   ((SmBezierAux2d*)pNewNode->m_pData)->m_eSplitDir = SM_SP_U;
                    pNext->Add(pNewNode);
                    pNewNode->m_pChild1 = pNode1;
                    pNewNode->m_pChild2 = pNode2;
                    pNode1->m_pParent = pNewNode;
                    pNode2->m_pParent = pNewNode;
                }
            }
          // If have odd number of rows add on last one
          if (lNumU % 2 == 1) 
            {
              for (ULONG kk=0; kk<lNumV; kk++) 
                {
                  ULONG lIndex = (lNumU-1)*lNumV + kk;
                  pNext->Add((*pCurr)[lIndex]);
                }
            }
          lNumU = (lNumU+1) / 2;  // Used up either half or one less rows
        }

      // Swap arrays
      SmTArray<SmTreeNode*> *pTmp = pCurr;
      pCurr = pNext;
      pNext = pTmp;
      pNext->ReSet();
    } // end While building baseNode parent generations loop

// GWCTreeVertexTemp
//  #ifdef SM_DEBUG_CODE
//    if(bDebugAssert)
//      {
//        SM_ASSERT_VALID(this) ;
//        SmBoolean bRtn = AssertValid() ;
//        SmBoolean bRtnTree = m_pTree->AssertTreeVertices() ;
//        m_pTree->DumpTreeVertices() ;
//        SM_ASSERT_BREAK(bRtn && bRtnTree) ;
//      }
//  #endif // SM_DEBUG_CODE

  // all done
  return SM_SUCCESS;

} // end SmSurfaceCache::SubdivideByBlock

/*******************************************************************//**
PURPOSE: Recursively Subdivide a node in the tree using 1 of 3 split strategies
            until tolerances have been achieved.

  1. m_bSwitchToFastSubdivision == TRUE and
     m_eTessellationAlgorithm   == SM_TA_FASTER_TESSELLATION and
     pNode->pAux->mBA_pSurface exists then
       tessellate this node into a set of nodes based on
       tessellation points picked by an adaptive tessellation
       of iso-parameter lines within the surface.
  2. m_bSwitchToFastSubdivision == TRUE and
     m_eTessellationAlgorithm   == SM_TA_FASTER_TESSELLATION and
     pNode->pAux->mBA_pSurface does not exists then
       Compute a Cubic Bezier Approximation to the current surface
       and subdivide that by splitting it evenly in 1/2 until
       each sub-surface patch satisfies the tessellation criteria
  3. Else
      Recursively subdivide the node exactly in 1/2 or at an existing node
      near the 1/2 mark until each sub-patch passes the tessellation 
      criteria.

NOTES: 
  1. Right now this only works for surfaces.  It may be
     enhanced to work with curves if needed.
  2. When a node is split
      a. two new nodes are created
      b. parent/child relations are set for every node.
      c. Split direction is saved in pNode->m_pData->m_eSplitDir
          //   with pNode->m_pData->m_eSplitDir = oneof SM_SP_U, SM_SP_V, where
          //          SM_SP_U: make new Child1 = left   (lower u half of parent domain)
          //                            Child2 = right  (upper u half of parent domain)
          //          SM_SP_V: make new Child1 = bot    (lower v half of parent domain)
          //                            Child2 = top    (upper v half of parent domain)
***********************************************************************/
SmStatus SmSurfaceCache::SubdivideToTolerances
  (SmTree        * pTree,         // i/o: Tree to modify
   SmTreeNode    * pNode,         // in : candidate for subdivsion
   SmMemBlockMgr * pBezierBlock,  // in : memory for gw_SURFACEs
                                  //      note: pNode->m_eAuxDataType equals either
                                  //            SM_AD_BEZIER_SURFACE (m_pData = (SmBezierPatch *)) or 
                                  //            SM_AD_AUX_OBJECT     (m_pData = (SmBezierAux2d *))
   ULONG         lRecursionDepth) // (for debugging)
{
#ifdef SM_DEBUG_CODE
static ULONG     slMaxDepth = 0;
SmBoolean bDebugMe2  = FALSE;
TCHAR            sBuff[SM_TBLOCK_SIZE];

  if ( lRecursionDepth > slMaxDepth ) 
    {
      slMaxDepth = lRecursionDepth;

      if ( bDebugMe2 ) 
        {
          smos_sprintf(sBuff, _T("SubdivideToTolerances() depth = %ld\n"), lRecursionDepth );
          smos_WriteBuffer(sBuff);
        }
    }
#endif // SM_DEBUG_CODE

  // Test to see if subdivision is required and pick a split direction
  //   note: TestAgainstTolerances is virtual and is used to do different 
  //         kinds of subdivision. Currently all classes subdivide on 
  //           ChordHeight
  //           AngleTolerance
  //           AspectRatio3D, 
  //           MaximumSideLength3D, and
  //           MinimumSideLength3D  tests when those tolerances are NonZero.
  //        
  //         class SmTessSrfCache does extra view dependent tests on  
  //           nodes spanning the silhouette curves when
  //           SmTessSrfCache::m_pSurfaceTessDriver->GetType() == SmViewBasedTessDriver_TYPE.
  SmBoolean       bSubdivideNode;
  SmSurfParamType eSubdivisionDirection;
  SER(TestAgainstTolerances(pNode,                   // in : node to test                             
                            bSubdivideNode,          // out: TRUE = patch fails some tessellation test
                            eSubdivisionDirection)); // out: suggested direction to split bad elements
                                                     //      oneof: SM_SP_U: Child1 = left, Child2 = right
                                                     //             SM_SP_V: Child1 = bot,  Child2 = top  
  // no work - patch currently within tolerance
  if (bSubdivideNode == FALSE)                 
    {
      return SM_SUCCESS;
    }

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe1 = FALSE;
  if(bDebugMe1)
    {
      SM_ASSERT_VALID(this) ;

      const SmSurface *cpSurface = GetSurface() ;
      SmFace  * pFace = (SmFace *)cpSurface->GetFace() ;
      SmBrep  * pBrep = pFace ? pFace->GetBrep() : NULL ; 
      SmPlane * pPlane = NULL ;
      cpSurface->GetNearPlaneForUVDraw(&pPlane) ; 
      SmObjDelete sClean(pPlane) ; 

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,1) ; if(cpSurface) cpSurface->DrawUV() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,4, 1,0,1) ; if(cpSurface) cpSurface->DrawNet() ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, .4,.4,.4) ; DrawSubdivision3D() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,4, 0,0,0) ; DrawSubdivision2D(TRUE, FALSE, FALSE, pPlane) ; sm_GraphicsLoop() ;  // with UVTrimCurves
      smgfx_SetLook(1,4, 0,0,0) ; DrawSubdivision2D(FALSE, FALSE, FALSE, pPlane) ; sm_GraphicsLoop() ; // without UVTrimCurves
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  if(bDebugMe)
    {
      SM_ASSERT_VALID(this) ;

      const SmSurface *cpSurface = GetSurface() ;
      SmFace  * pFace = (SmFace *)cpSurface->GetFace() ;
      SmPlane * pPlane = NULL ;
      cpSurface->GetNearPlaneForUVDraw(&pPlane) ; 
      SmObjDelete sClean(pPlane) ; 

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,0) ; if(pFace) pFace->DrawSubdivisions(TRUE) ; sm_GraphicsLoop() ; // option: vary tess args to see different tessellations
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE


  // Switch between subdivision strategies.
  //
  // No: the following is obsolete.
  // For tests other than max-3d-edge-length, SubdivideByBlock() looks at
  // only the boundary curves, and possibly the diagonals, of the given
  // surface subdomain, and so might not work well if the patch is big
  // enough to have 'surprises' in its interior -- even if max-3d-edge
  // is specified.
  // We will let TestAgainstTolerances() make the decision, and abide by it.
  // [B80]
  //
  // Obsolete:
  // // Note: Do this regardless of this switch.
  // // SubdivideByBlock() is faster and produces better results than
  // // bisection, particularly when max-3d-edge length is specified,
  // // and it now works on patches that are not almost flat.  [bd 9/30/10, B71]

  if(   m_bSwitchToFastSubdivision // default:[FALSE] - set to TRUE in TestAgainstTolerances
                                   //   if turning angle gets below m_dStartFastSubdivisionAngleDeg
                                   //   and m_eTessellationAlgorithm == SM_TA_FASTER_TESSELLATION
     && m_eTessellationAlgorithm == SM_TA_FASTER_TESSELLATION    // constructor default:[SM_TA_FASTER_TESSELLATION]
     && ((SmBezierAux2d*)pNode->m_pData)->mBA_pSurface != NULL)  // from SmTessSrfCache::BuildTreeWithSubdivision(), mBA_pSurface = Surface Copy
    {                                                            //      SmSurfaceCache::BuildTreeBase(),            mBA_pSurface = NULL        

      SM_ASSERT(pNode->m_eAuxDataType == SM_AD_AUX_DATA) ; 

      // tessellate this node completely in a single pass
      //   based on tessellation break points from u and v iso-parameter line tessellations  
      //   making an array of basenodes which get connected together into 
      //   a binary tree of parents where each parent's domain is the union of its children's.
      // These parents don't get a 3d bounding box.
      SER(SubdivideByBlock(pTree, pNode));
      m_bSwitchToFastSubdivision = FALSE;

      return SM_SUCCESS;
    } // end SubdivideByBlock branch
  // else subdivide node into two branch

  //      if(   m_bSwitchToFastSubdivision                              // default:[FALSE]
  //         && m_eTessellationAlgorithm == SM_TA_FASTER_TESSELLATION   // default:[SM_TA_FASTER_TESSELLATION]
  //         && ((SmBezierAux2d*)pNode->m_pData)->mBA_pSurface == NULL) // from SmSurfaceCache::BuildTreeBase(),            mBA_pSurface = NULL
  //        {                                                           //      SmTessSrfCache::BuildTreeWithSubdivision(), mBA_pSurface = Surface Copy
  //          ERR_MSG(_T("GWC: Dead Surface Subdivision Branch was called - need to see why")) ;
  //          return SM_SUCCESS;
  //      
  //          // GWC:TEST - I think this might be dead code
  //          // because m_bSwitchToFastSubdivision == TRUE only for calls from
  //          //   BuildTreeWithSubdivision() where mBA_pSurface != NULL.  so I'll test it.
  //          // Tested: Never called as suspected.
  //      
  //          //      SmCubicBezierSurface sBez1, sBez2;
  //          //      SER(m_cpSurface->ComputeCubicBezierApprox(pAux->m_sUVDomain,sBez1));
  //          //      // subdivide the node - at mid-point
  //          //      SER(SubdivideNodeFast(pTree,pNode,eSubdivisionDirection,sBez1,sBez1,sBez2));
  //          //      SER(SubdivideToTolerancesFast(pTree,pNode->m_pChild1,sBez1));
  //          //      SER(SubdivideToTolerancesFast(pTree,pNode->m_pChild2,sBez2));
  //          //      m_bSwitchToFastSubdivision = FALSE;
  //          //      
  //          //      return SM_SUCCESS;
  //        } // end dead code branch
     
  // else (   m_bSwitchToFastSubdivision == FALSE                       // constructor default:[FALSE]                    
  //       || m_eTessellationAlgorithm   != SM_TA_FASTER_TESSELLATION)  // constructor default:[SM_TA_FASTER_TESSELLATION]
  //

  // Subdivide the node in two - at mid-point or existing knot near mid-point when possible.
  //   with eSubdivideDirection = one of SM_SP_U, SM_SP_V, where
  //          SM_SP_U: make new Child1 = left
  //                            Child2 = right
  //          SM_SP_V: make new Child1 = bot
  //                            Child2 = top
                      
#ifdef SM_DEBUG_CODE
  if (bDebugMe) 
    {
      pTree->Dump() ;
    }
#endif // SM_DEBUG_CODE

  SER(SubdivideNode(pNode,                    // in : target node                                                  
                    eSubdivisionDirection,    // in : target node->m_pData->m_eSplitDir = eSubdivideDirection, and 
                                              //      SM_SP_U: Child1 = left                                       
                                              //               Child2 = right                                      
                                              //      SM_SP_V: Child1 = bot                                        
                                              //               Child2 = top                                        
                                              //      
                    FALSE,                    // in : FALSE=   Bezier patch shape to split
                                              //      TRUE =no Bezier patch shape to split                          
                                              //      set to TRUE when inserting edgeuses into the subdivision Tree
                    pBezierBlock)) ;          // in : memory for gw_SURFACE bezier patches

#ifdef SM_DEBUG_CODE
  if (bDebugMe) 
    {
      pTree->Dump() ;
    }
#endif // SM_DEBUG_CODE

  // recursion - Keep subdividing until we satisfy tolerances      
  SER(SubdivideToTolerances(pTree,pNode->m_pChild1, pBezierBlock, lRecursionDepth+1 ));
  SER(SubdivideToTolerances(pTree,pNode->m_pChild2, pBezierBlock, lRecursionDepth+1 ));

  // We get a minimal bounding box by unioning children's boxes and
  // propagating up the tree.

  // Bounding Box union - This eliminates errors due to
  //    1. computing bounding boxes from BezierPatches constructed from surface approximations of nonBSplineSurfaces, or
  //    2. tolerance accumulation for bounding boxed from BezierPatches made for BSplineSurfaces.
  // for the coarse approximations made for non-SmBSplineSurfaces.
  pNode->m_pChild1->m_sBBox.Union(pNode->m_pChild2->m_sBBox,pNode->m_sBBox);

  SM_ASSERT_MSG(pNode->m_pChild1->m_sBBox.IsContainedBy(pNode->m_sBBox, SM_EFF_ZERO_SQRT),_T("pChild1 BBox not contained in parent BBox")) ;
  SM_ASSERT_MSG(pNode->m_pChild2->m_sBBox.IsContainedBy(pNode->m_sBBox, SM_EFF_ZERO_SQRT),_T("pChild2 BBox not contained in parent BBox")) ;

  // When BezierPatch approximations are being made for non BSplineSurfaces
  if (   pNode->m_eAuxDataType == SM_AD_BEZIER_SURFACE
      // && bOnlySubdivideAuxData == FALSE                  // this line always TRUE do to above SubdivideNode() call's 'FALSE' argument
      && !m_cpSurface->IsKindOf(SmBSplineSurface_TYPE))
    {
      // need to make sure the parent BBoxes are the union of the children
      // to avoid coarse approximation errors

      SM_ASSERT(pNode->m_eAuxDataType            == SM_AD_BEZIER_SURFACE) ;
      SM_ASSERT(pNode->m_pChild1->m_eAuxDataType == SM_AD_BEZIER_SURFACE) ;
      SM_ASSERT(pNode->m_pChild2->m_eAuxDataType == SM_AD_BEZIER_SURFACE) ;

      // Get Node Aux Data
      SmBezierPatch *pBez  = (SmBezierPatch*)pNode->m_pData ;
      SmBezierPatch *pBez1 = (SmBezierPatch*)pNode->m_pChild1->m_pData ;
      SmBezierPatch *pBez2 = (SmBezierPatch*)pNode->m_pChild2->m_pData ;

      // Set Parent Pseudo and Polar BBoxes = Union(Child BBoxes)
      pBez1->GetPseudoBox().Union(pBez2->GetPseudoBox(), pBez->GetPseudoBox().GetBasis(), pBez->GetPseudoBox()) ;
      //pBez1->GetPolarBox().Union (pBez2->GetPolarBox(),  pBez->GetPolarBox()) ;
    }  
  // else // not building BezierPatch approximations to non BSplineSurfaces, so
  //   { 
  //      no need to union pseudo boxes - they don't nest, and without coarse approximations they don't need to be corrected
  //      no need to union polar boxes  - they don't nest, and without coarse approximations they don't need to be corrected
  //   }

  // all done
  return SM_SUCCESS;

} // end SmSurfaceCache::SubdivideToTolerances

// GWC removed unused function
//      
//      /*******************************************************************//**
//      PURPOSE: Subdivide a node in the tree by splitting it in 1/2 repeatedly 
//                  until tolerances have been achieved.
//      
//      NOTES: Right now this only works for surfaces.  It may be
//         enhanced to work with curves if needed.
//      ***********************************************************************/
//      SmStatus SmSurfaceCache::SubdivideToTolerancesFast
//        (SmTree *pTree,
//         SmTreeNode *pNode,
//         SmCubicBezierSurface & rBezier)
//      {
//          SmBoolean bSubdivideNode;
//          SmSurfParamType eSubdivisionDirection;
//      
//          // Test to see if subdivision is required
//          SER(TestAgainstTolerancesFast(pTree,pNode,rBezier,bSubdivideNode,eSubdivisionDirection));
//      
//          if (bSubdivideNode == FALSE) {
//              return SM_SUCCESS;
//          }
//      
//          SmCubicBezierSurface sBez2;
//          //SmBezierAux2d *pAux = (SmBezierAux2d*)pNode->m_pData;
//          SER(SubdivideNodeFast(pTree,pNode,eSubdivisionDirection,rBezier,rBezier,sBez2));
//          SER(SubdivideToTolerancesFast(pTree,pNode->m_pChild1,rBezier));
//          SER(SubdivideToTolerancesFast(pTree,pNode->m_pChild2,sBez2));
//      
//          // We get a minimal bounding box by unioning children's boxes and
//          // propagating up the tree.
//          pNode->m_pChild1->m_sBBox.Union(pNode->m_pChild2->m_sBBox,pNode->m_sBBox);
//      
//          return SM_SUCCESS;
//      
//      } // end SmSurfaceCache::SubdivideToTolerancesFast

/********************************************************************
PURPOSE: Create a block of bezier patch objects from a BSpline Surface.
    The set of Bezier patches is a decomposition of the BSpline Surface.

NOTES:
  1. Allocates and builds one Bezier patch for every non-zero Surface span
      Although not specified, the patches are currently odered as
         [patch00, patch01, ... patch0N, patch10, ... patchMN]
  2. all bezier patch memory is allocated in block of memory contained in rBezierMgr
  3. The output is rBeziers, a list of BezierPatch pointers whose 
     values all point to the memory allocated within rBezierMgr.
*********************************************************************/
static SmStatus sm_CreateBezierPatchBlock
  (const gw_SURFACE       * pSource,      // in : BSpline surface to decompose      
   SmMemBlockMgr          & rBezierMgr,   // out: modified to hold memory for array of bezier patches, stores SmBezierPatch objects
   SmMemBlockMgr          * pBezierBlock, // in : Bezier Surface Patch memory, stores gw_SURFACE objects   
   ULONG                  & rlNumUSpans,  // out: number of u spans in pSource
   ULONG                  & rlNumVSpans,  // out: number of v spans in pSource
   SmTArray<gw_SURFACE *> & rBeziers)     // out: list of Bezier patch pointers - one for every non-zero surface span
                                          //      ordered:[patch00, patch01, ... patch0N, patch10, ... patchMN]
{
  // BSpline Surface locals               
  gw_KNOTVECTOR *knu = pSource->knu;
  gw_KNOTVECTOR *knv = pSource->knv;

  // get span counts
  gw_INDEX nspu, nspv;
  N_BasisGetSpanCount(knu,pSource->p,&nspu);
  N_BasisGetSpanCount(knv,pSource->q,&nspv);

  // set output
  rlNumUSpans = nspu;
  rlNumVSpans = nspv;

  // compute size in bytes of the BSpline Surface
  ULONG lBezSize = sm_ComputeNurbSurfaceSize(pSource->p,     pSource->q,
                                             pSource->p*2+1, pSource->q*2+1);

  // Figure out how many gw_SURFACE bezier patches per allocated block.
  // Don't let this get too big because that wastes a lot space.
  ULONG lNumPerBlock = rlNumUSpans * rlNumVSpans + pSource->p + pSource->q + 1 ;
  if (lNumPerBlock > 40) 
    {
      lNumPerBlock = 40;
    }
  // initilize bezier patch memory to hold SmBezierPatch objects augmented to hold the gw_SURFACE values.
  rBezierMgr   .Initialize(ALIGN_SIZE(sizeof(SmBezierPatch)), lNumPerBlock);
  pBezierBlock->Initialize(             lBezSize,             lNumPerBlock) ;

  // allocate enough room to hold a Bezier patch for every span cross-product plus a few
  rBeziers.SetSize(rlNumUSpans*rlNumVSpans);

  // for every surface patch - build rBeziers list, init pointer values
  for (ULONG i=0; i<rlNumUSpans*rlNumVSpans; i++) 
    {
      // get pointer to memory to hold Bezier patch
      SmBezierPatch *pBezPat  = (SmBezierPatch*)rBezierMgr.GetAt(i);
      pBezPat->m_pBezier = (gw_SURFACE *)pBezierBlock->GetAt(i);

      gw_SURFACE    *pSurface = pBezPat->GetBezierPtr();

      // initialize pSurface's (the new bezier patch) internal pointers to mimick the 
      //   NLIB data structure but all in one contiguous piece of memory.
      // note: this sets pointers - but does not set any of the data.
      sm_InitNurbSurfaceMemory(pSurface, 
                               pSource->p, pSource->q,
                               pSource->p, pSource->q,
                               pSource->p*2+1,pSource->q*2+1);
      
      // store the bezier patch object in the list of bezier patches
      rBeziers[i] = pSurface;
    } // end iter every surface patch initializing pointer addresses

  // store a bezier patch for every non-zero span
  // order: [patch00, patch01, ... patch0N, patch10, ... patchMN]
  GW_SER(sm_DecomposeSrf(pSource, rBeziers));

  // all done
  return SM_SUCCESS;

} // end sm_CreateBezierPatchBlock

/********************************************************************
PURPOSE:

NOTES:
*********************************************************************/
static SmStatus sm_CreateApproximateBezierPatchBlock
  (const SmSurface        & crSurface,     // in : Non-BSpline surface to decompose                                       
   SmMemBlockMgr          & rBezierMgr,    // out: modified to hold memory for array of SmBezierPatch objects 
   SmMemBlockMgr          * pBezierBlock,  // out: modified to hold memory for array of NURB gw_SURFACE objects               
   ULONG                  & rlNumUSpans,   // out: number of u spans in pSource                                       
   ULONG                  & rlNumVSpans,   // out: number of v spans in pSource 
   SmTArray<double>       & rUSplits,      // out: Split Points in U direction                                      
   SmTArray<double>       & rVSplits,      // out: Split Points in V direction                                      
   SmTArray<gw_SURFACE *> & rBeziers)      // out: list of Bezier patch pointers - one for every non-zero surface span
                                           //      ordered:[patch00, patch01, ... patch0N, patch10, ... patchMN]      
{
  // Note we are creating approximate cubic beziers
  //   - this can be used to reduce the cache size for an SmBSplineSurface 
  //                      or for an SmOffsetSurface
  double dKnot ;
  SER(crSurface.GetKnots(SM_SP_U,rUSplits));

  // limit initial U knot count to 6
  if (rUSplits.GetSize() > 6) 
    {
      double dMin = rUSplits[0];
      double dMax = rUSplits.GetLast();        
      rUSplits.SetSize(6);
      rUSplits.SetAt(0, dMin) ;
      rUSplits.SetAt(5, dMax) ;
      for (ULONG i=1; i<=4; i++) 
        {
          dKnot = dMin + (i / 5.0) * (dMax - dMin);
          rUSplits.SetAt(i, dKnot);
        }
    }

  // limit initial V knot count to 6
  SER(crSurface.GetKnots(SM_SP_V,rVSplits));
  if (rVSplits.GetSize() > 6) 
    {
      double dMin = rVSplits[0];
      double dMax = rVSplits.GetLast();        
      rVSplits.SetSize(6);
      rVSplits.SetAt(0, dMin) ;
      rVSplits.SetAt(5, dMax) ;
      for (ULONG i=1; i<=4; i++) 
        {
          dKnot = dMin + (i / 5.0) * (dMax - dMin);
          if (i == 5) dKnot = dMax;
          rVSplits.SetAt(i, dKnot);
        }
    }

  // get non-zero span count
  rlNumUSpans = rUSplits.GetSize() - 1;
  rlNumVSpans = rVSplits.GetSize() - 1;

  // get memory size in bytes of one degree 3x3 gw_SURFACE
  ULONG lBezSize     = sm_ComputeNurbSurfaceSize(3,3,3*2+1,3*2+1);
  ULONG lNumPerBlock = rlNumUSpans * rlNumVSpans + 16;
  SM_ASSERT(lNumPerBlock <= 5*5 + 16) ;

  // Don't let lNumPerBlock get too big because it wastes space
  if (lNumPerBlock > 100) lNumPerBlock = 100;
  if (lNumPerBlock <  30) lNumPerBlock =  30;

  // initilize bezier patch memory to hold SmBezierPatch objects augmented to hold the gw_SURFACE values.
  rBezierMgr.   Initialize(ALIGN_SIZE(sizeof(SmBezierPatch)), lNumPerBlock);
  pBezierBlock->Initialize(            lBezSize,              lNumPerBlock);

  ULONG lCount = 0;
  rBeziers.SetSize(rlNumUSpans*rlNumVSpans);

  // for every U span
  for (ULONG i=0; i<rlNumUSpans; i++) 
    {
      double dMinU = rUSplits[i];
      double dMaxU = rUSplits[i+1];

      // for every V Span
      for (ULONG j=0; j<rlNumVSpans; j++) 
        {
          double          dMinV   = rVSplits[j];
          double          dMaxV   = rVSplits[j+1];

          // set next patch extent
          SmExtent2d sUVDomain(dMinU, dMinV, 
                               dMaxU, dMaxV);
          
          // get next patch memory from rBezierMgr
          SmBezierPatch * pBezPat = (SmBezierPatch*)rBezierMgr.GetAt(lCount);
          pBezPat->m_pBezier = (gw_SURFACE *)pBezierBlock->GetAt(lCount);

          gw_SURFACE    * pSurface = pBezPat->GetBezierPtr();

          // set next patch memory internal pointers to mimick NLib structure but in contiguous memory
          sm_InitNurbSurfaceMemory(pSurface,3,3,3,3,3*2+1,3*2+1);

          // approximate the surface shape with a degree 3x3 bezier patch
          sm_ComputeBezierApproximation(crSurface,sUVDomain,pSurface);

          // make the surface pointer array
          rBeziers[lCount] = pSurface;
          lCount ++;
        }
    } // end iter every USpan making a degree 3x3 bezier approximation
    
  // all done 
  return SM_SUCCESS;

} // end sm_CreateApproximateBezierPatchBlock

/*******************************************************************//**
PURPOSE: Build the base level of the tree and do subdivision down.

NOTES ---
  start m_pTree with a set of m_lNumUSpans X m_lNumVSpans subdivision starter nodes 
       each with its own SmBezierPatch Data Object containing a gw_SURFACE object.
  
    if(m_cpSurface is  SmBSPlineSurface) - Starter nodes = nonZero domain spans
    if(m_cpSurface not SmBsplineSurface) - Starter nodes = 5x5 (or less) grid of approximate BezierPatches
  
  Then each starter node is subdivided into children nodes until
      every child node's Bezier Patch passes the tolerance requirements.

OUTPUT ---
  1. rNodes = list of starter Nodes, sized:[m_lNumUSpans * m_lNumVSpans]

SIDE EFFECTS --- 
  1. pSurfaceCache->m_pTree             = new SmTree 
  2. pSurfaceCache->m_sBezMgr allocates memory to hold 1 bezier patch
                                          for every non-zero surface span.
  3. pSurfaceCache->m_pTree->m_sNodeMgr = memory for 1 TreeNode for each Bezier patch
  4. pSurfaceCache->m_pTree->m_sListMgr = memory for 1 SmObjectList for each Node

  Every new TreeNode:
    pTreeNode->m_eAuxDataType = SM_AD_BEZIER_SURFACE
    pBezierPatch->mBA_pSurface  = NULL
    pBezierPatch->m_sUVDomain = gw_SURFACE->Domain
***********************************************************************/
SmStatus SmSurfaceCache::BuildTreeBase
  (SmTArray<SmTreeNode*> & rNodes,                 // out: array of tree leaf nodes
   ULONG                 & rl1stGenerationUCount,  // out: number of U entries in rNodes
   ULONG                 & rl1stGenerationVCount,  // out: number of V entries in rNodes
   SmMemBlockMgr         * pBezierBlock)           // in : Bezier Surface Patch memory
{
  // locals
  SmSurfaceCache * pSurfaceCache = this;
  SM_ASSERT(pSurfaceCache != NULL);

  // store an allocated and empty decomposition tree in this->m_pTree
  m_pTree       = new (*GetContext()) SmTree(this);
  SmTree *pTree = m_pTree;    // store m_pTree pointer for debug purposes

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  if (bDebugMe) 
    {
      // dump and draw tgt surface.  Note: draw side effect may build the cache - watch for side effects
      m_cpSurface->Dump();

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,1,1) ; if(m_cpSurface) m_cpSurface->Draw(); sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // decomposition locals
  SmTArray<double> sUSplits, sVSplits ;
  
  // First, decompose the surface into starter Bezier patches.
  // These end up being the decomposition-tree's 1st generation of leaf-node shapes.
  //     side effects: 1. load m_sBezMgr with ordered set of Bezier Patches stored as gw_SURFACES
  //                                     ordered[patch_00, 01, .. 0N, 10, 11, ... MN]
  //                   2. load sBeziers  to point into the m_sBezMgr gw_SURFACES
  //                                     sBeziers[i] = m_sBezMgr.GetAt(i)
  SmTArray<gw_SURFACE*> sBeziers(256);
  if (m_cpSurface->IsKindOf(SmBSplineSurface_TYPE)) 
    {
      SmBSplineSurface & rSurface = (SmBSplineSurface&)*pSurfaceCache->m_cpSurface ;

      // Get the UniqueKnot values - they're being used as the split points in sm_CreateBezierPatchBlock
      rSurface.GetKnots(SM_SP_U, sUSplits) ;
      rSurface.GetKnots(SM_SP_V, sVSplits) ;

      // create an array of bezier patches - one for every every non-zero surface span
      SmStatus sRtn = sm_CreateBezierPatchBlock
              (rSurface.m_pNurb,              // in : tgt Nurb surface
               pSurfaceCache->m_sBezMgr,      // in : to hold SmBezierPatch objects
               pBezierBlock,                  // in : to hold gw_SURFACE objects
               rl1stGenerationUCount,         // out: number of U spans,
               rl1stGenerationVCount,         // out: number of V spans,
               sBeziers) ;                    // out: list of Bezier patch pointers - one for every non-zero surface span
                                              //      ordered:[patch00, patch01, ... patch0N, patch10, ... patchMN]
      SER(sRtn) ;
    }
  else // not a BSplineSurface type
    {
      // create an array of approximate degree 3x3 bezier patches - 
      //    one for every every non-zero surface span up until the span count gets > 5
      //    then build 5 evenly spaced degree 3x3 bezier surface approximations to the real surface
      // gwc:todo - use a better approximation algorithm.
      SmStatus sRtn = sm_CreateApproximateBezierPatchBlock
              (*pSurfaceCache->m_cpSurface,    // in : tgt surface
               pSurfaceCache->m_sBezMgr,      // in : to hold SmBezierPatch objects
               pBezierBlock,                  // in : to hold gw_SURFACE objects
               rl1stGenerationUCount,         // out: number of U spans,
               rl1stGenerationVCount,         // out: number of V spans,
               sUSplits,                      // out: Split Points in U direction
               sVSplits,                      // out: Split Points in V direction
               sBeziers);                     // out: list of Bezier patch pointers - one for every non-zero surface span
                                              //      ordered:[patch00, patch01, ... patch0N, patch10, ... patchMN]
      SER(sRtn) ;
    }

  // arrive here with 1st subdivision tree generation built in m_sBezMgr and pointed to by sBeziers.

#ifdef SM_DEBUG_CODE
  // draw Brep, Surface, Face, and sBeziers 1st generation
  //  note: drawing the surface will build the cache - so watch out for the state change here.
  if(bDebugMe)
    {
      const SmSurface *cpSurface = this->GetSurface() ;
      SmFace          *pFace     = (SmFace *)cpSurface->GetFace() ;
      SmBrep          *pBrep     = pFace ? pFace->GetBrep() : NULL ;

      smgfx_Erase() ;
      smgfx_SetLook(1,2, .1,.5,.8) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, .3,.7,.1) ; if(cpSurface) cpSurface->DrawUV(7,7) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,0) ; if(pFace) pFace->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop() ;
      for(ULONG di=0;di<sBeziers.GetSize();di++) { SmBSplineSurface sBSplineSurface(sBeziers[di], TRUE, m_cpContext) ; // was TRUE = don't run AssertValidAndHeal on New BSplineSurface
        smgfx_ChangeColor(di != 0) ; sBSplineSurface.DrawUV(5,5) ; sm_GraphicsLoop() ; }
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // Initialize node block manager
  //   lPatchCount = rl1stGenerationUCount * rl1stGenerationVCount;
  //   lNumElem    = min number of elements per allocate block for memory managers
  ULONG lNumElem = 30; // smos_Min((3 * lPatchCount) + 4,30);

  // m_sBezMgr has been initialized - now init m_sNodeMgr and m_sAuxMgr memory managers

  // remember the node size     and the number of nodes to allocated as a block
  // remember the Aux Data size and the number of nodes to allocated as a block
  pTree->m_sNodeMgr.Initialize(ALIGN_SIZE(sizeof(SmTreeNode)),    lNumElem);
  m_sAuxMgr.Initialize        (ALIGN_SIZE(sizeof(SmBezierAux2d)), lNumElem);

  // Set the continuities and compute initial tree nodes and other node data.
  //   Note: continuity is in opposite direction as to that which it was queried.
  SmContinuityType eMinCont;
  ULONG lNodeCount=0;
  SmTArray<SmContinuityType> & rUContinuities = *pSurfaceCache->m_pUContinuities;
  SmTArray<SmContinuityType> & rVContinuities = *pSurfaceCache->m_pVContinuities;
  SER(pSurfaceCache->m_cpSurface->CalculateContinuities(SM_SP_U, eMinCont, rUContinuities));
  SER(pSurfaceCache->m_cpSurface->CalculateContinuities(SM_SP_V, eMinCont, rVContinuities));

  // pre-allocate enough memory to hold all the leaf nodes but set array count to zero
  rNodes.SetSize(  rl1stGenerationUCount    //   pSurfaceCache->m_lNumVSpans
                 * rl1stGenerationVCount);  // * pSurfaceCache->m_lNumUSpans);
  rNodes.ReSet();

// GWCTreeVertexTemp
//  // Layout the TreeVertex indexing scheme
//  SmTArray<ULONG> sTreeVertexMap, sTreeVertexPre, sTreeVertexPost ;
//  pTree->BuildTreeVertexMaps
//    (NULL,               // in : Parent Node being split before TreeVertices entries are updated or NULL for none
//     sUSplits,           // in : rUSplits[i] = ith Parent Node Split Point with bndrys. Size of 2=no splits, 3=1 split, . . .
//     sVSplits,           // in : rVSplits[j] = jth Parent Node Split Point with bndrys. Size of 2=no splits, 3=1 split, . . .
//     sTreeVertexMap,     // out: Index for every U/V split TreeVertex intersection
//     sTreeVertexPre,     // out: closest TreeVertexIndex to This between This and Last U/V SplitPoints, 
//                         //      SM_BIG_ULONG = no TreeVertices between This and Last U/V SplitPoint
//     sTreeVertexPost) ;  // out: closest TreeVertexIndex to This between This and Next U/V SplitPoints, 
//                         //      SM_BIG_ULONG = no TreeVertices between This and Next U/V SplitPoint

// GWCTreeVertexTemp
//  #ifdef SM_DEBUG_CODE
//     if(bDebugMe)
//       {
//         pTree->DumpTreeVertices() ;
//       }
//  #endif // SM_DEBUG_CODE

  // connect every subdivided bezier patch to a tree node - and set all SmBezierPatch data  
  for (ULONG ii=0; ii<rl1stGenerationUCount; ii++) 
    {
      for (ULONG jj=0; jj<rl1stGenerationVCount; jj++) 
        {
          // get next node and Bezier Patch (to be connected)
          SmTreeNode    *pNode = (SmTreeNode*)pTree->m_sNodeMgr.GetAt(lNodeCount++);
          SmBezierPatch *pBez  = pSurfaceCache->GetPatchAt(ii, jj, rl1stGenerationUCount, rl1stGenerationVCount);
          pBez->m_pBezier      = (gw_SURFACE *)pBezierBlock->GetAt(ii*rl1stGenerationVCount + jj);
          pBez->m_pOwningTreeNode = pNode;

          // add this node's SmTreeNode Memory to rNodes array
          rNodes.Add(pNode);

          // Build a BSplineSurface for the BezierPatch geometry
          SmBSplineSurface sBSS(pBez->GetBezierPtr(), TRUE, m_cpSurface->GetContext()) ;

          // initialize all pNode data - connect pNode to pBez
          pNode->m_pTree   = pTree ;
          pNode->m_pParent = NULL;
          pNode->m_pChild1 = NULL ;
          pNode->m_pChild2 = NULL ;

          pNode->m_eAuxDataType = SM_AD_BEZIER_SURFACE;
          pNode->m_pData        = pBez;
          pNode->m_eGeomType    = SM_NG_DEFAULT;

#ifdef SM_DEBUG_CODE
          if (bDebugMe) 
            {
              sBSS.Dump();

              // add this surface patch to ouput graphics
              smgfx_SetLook(1,2, 0,0,1) ; sBSS.DrawUV(5,8); sm_GraphicsLoop() ;
              sm_GraphicsLoop() ;
            }
#endif // SM_DEBUG_CODE

          // now, initialize all pBez memory

          // compute and store TreeNode->BoundingBox, BezierPatch->PseudoBox
          SER(sBSS.CalculateBoundingBox(sBSS.GetNaturalUVDomain(), &pNode->m_sBBox, &pBez->m_sPseudoBox, NULL));

          SM_ASSERT_MSG(   pNode->m_pParent == NULL
                        || pNode->m_sBBox.IsContainedBy(pNode->m_pParent->m_sBBox, SM_EFF_ZERO_SQRT),_T("pChild1 BBox not contained in parent BBox")) ;

          // set patch continuity to its neighbor values 
          pBez->m_eVCurveConts[0] = (SmContinuityType)rVContinuities[jj];
          pBez->m_eUCurveConts[0] = (SmContinuityType)rUContinuities[ii];
          pBez->m_eVCurveConts[1] = (SmContinuityType)rVContinuities[jj+1];
          pBez->m_eUCurveConts[1] = (SmContinuityType)rUContinuities[ii+1];

          // initialize all remaining SmBezierPatch data  
          pBez->m_eSplitDir  = SM_SP_U;   // GWC SHORTSURFACECACHE: should init to SM_SP_UNKNOWN
          pBez->m_bIsMarked  = FALSE;
          pBez->m_eNodeClass = SM_NC_UNKNOWN;

          pBez->m_pFace      = NULL;
          pBez->mBA_pSurface = NULL;

          pBez->m_bChordHeightSatisfied = FALSE;
          pBez->m_bAngleTolSatisfied    = FALSE;
          pBez->m_sUVDomain             = sBSS.GetNaturalUVDomain();

          pBez->m_vVertexList.Init();
          pBez->m_sEdgeuseList.Init();
          pBez->m_sPolyEdgeList.Init();

// GWCTreeVertexTemp
//          // update the Tree's TreeVertex connectivity graph
//          pBez->m_lStartTreeVertexIndx = pTree->SetTreeVertexLoop
//               (pNode,                    // in : TreeNode owner of this TreeVertex Loop
//                pBez->m_sUVDomain,        // in : UVDomain for Node with this loop
//                ii, jj+1,                 // in : Above Left Corner index[ii,jj] of new Node[ii,jj] TreeVertex Loop
//                sUSplits, sVSplits,       // in : Total Number of U and V Splits in TreeVertex Array(ii,jj)
//                sTreeVertexMap,           // in : Array made by BuildTreeVertexMaps call
//                sTreeVertexPre,           // in : Array Made By BuildTreeVertexMaps call
//                sTreeVertexPost) ;        // in : Array Made By BuildTreeVertexMaps call
//            
//  #ifdef SM_DEBUG_CODE
//            if(bDebugMe)
//              {
//                pTree->DumpTreeVertices() ;
//              }
//  #endif // SM_DEBUG_CODE

        } // end iter every Surface non-zero V Span
    } // end iter every subdivided bezier patch

// GWCTreeVertexTemp
//   #ifdef SM_DEBUG_CODE
//     if(bDebugAssert)
//       {
//         SmBoolean bRtn = m_pTree->AssertTreeVertices() ;
//         m_pTree->DumpTreeVertices() ;
//         SM_ASSERT_BREAK(bRtn) ;
//       }
//   #endif // SM_DEBUG_CODE

  // Subdivide each 1st generation surface node into pieces that all meet tessellation requirements.
  // note: SmTessSrfCache::BuildTreeWithSubdivision() differs from 
  //       SmSurfaceCache::BuildTreeBase().
  //        - SmTessSrfCache::BuildTreeWithSubdivision() 
  //             calls SubdivideToTolerances() 
  //             - with one node encompassing the whole surface
  //             - and pNode->m_eAuxDataType == SM_AD_AUX_DATA       (m_pData = (SmBezierAux2d *))
  //        - SmSurfaceCache::BuildTreeBase() 
  //            calls SubdivideToTolerances() 
  //             - once for every member of an array of nodes encompassing pieces of the whole surface
  //             - and pNode->m_eAuxDataType == SM_AD_BEZIER_SURFACE (m_pData = (SmBezierPatch *))

  // for every 1st generation surface subdivision
  for (ULONG ii=0; ii<lNodeCount; ii++) 
    {
      SmTreeNode *pNode = (SmTreeNode*)pTree->m_sNodeMgr.GetAt(ii);

      // Subdivide each original surface Patch (one for every non-Zero surface span)
      //  until all its children pass the given tolerance criteria
      SER(SubdivideToTolerances(pTree, pNode, pBezierBlock));

#ifdef SM_DEBUG_CODE
      // draw 
      if(bDebugMe)
        {
          const SmSurface *cpSurface = this->GetSurface() ;
          SmFace          *pFace     = (SmFace *)cpSurface->GetFace() ;
          SmBrep          *pBrep     = pFace ? pFace->GetBrep() : NULL ;

          SmTArray<SmTreeNode *> sOffspring ;
          pNode->GetOffspring(sOffspring) ;

          pTree->Dump() ;

          smgfx_Erase() ;
          smgfx_SetLook(1,2, .1,.5,.8) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, .3,.7,.1) ; if(cpSurface) cpSurface->DrawUV(7,7) ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 0,0,0)    ; if(pFace) pFace->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop() ;

          smgfx_SetLook(1,2, .7,.3,.4) ; pNode->m_sBBox.Draw() ; sm_GraphicsLoop() ;
          for(ULONG di=0;di<sOffspring.GetSize();di++)  
            { smgfx_ChangeColor(di != 0) ; sOffspring[di]->m_sBBox.Draw() ; sm_GraphicsLoop() ; }
          sm_GraphicsLoop() ;
        }
#endif // SM_DEBUG_CODE

    } // end iter every 1st generation patch to make recursive subdivisions to meet tolerances

// GWCTreeVertexTemp
//  #ifdef SM_DEBUG_CODE
//    if(bDebugAssert)
//      {
//        pTree->Dump() ;
//        SmBoolean bRtn = m_pTree->AssertTreeVertices() ;
//        m_pTree->DumpTreeVertices() ;
//        SM_ASSERT_BREAK(bRtn) ;
//      }
//  #endif // SM_DEBUG_CODE

  // all done
  return SM_SUCCESS;

} // end SmSurfaceCache::BuildTreeBase

/*******************************************************************//**
PURPOSE: Tessellate a surface using either chord height and/or angular
    tessellation tolerance and produce a Bezier representation of the surface.

NOTES: If one of the tolerances are zero then that tolerance is
    not used in the calculation.  Both of the tolerances must not be zero.
   At least one of the outputs must be a non-NULL pointer.

   2. Trees are built for BSplineSurface and non BSplineSurface type surfaces.
      Subdivision starts with the knot boundaries for BSplineSurfaces, and
      a set of 5x5 (or less) approximating degree 3x3 bezier patches for
      non BSplineSurfaces.
***********************************************************************/
SmStatus SmSurfaceCache::BuildTree
  (SmMemBlockMgr *pOptBezierBlock)    // in : notNULL = called from a derived class during tree construction
                                      //        do not set SmBezierPatch::mBA_pSurface values.
                                      //          to be finished by the derived classes.
                                      //      NULL    = called for SmSurfaceCache construction.
                                      //        do set SmBezierPatch::mBA_pSurface values.
                                      //      default:[NULL]
{
  //std::lock_guard<std::recursive_mutex> lock((GetSurface().mCacheMutex));

  // locals
  SmTArray<SmTreeNode*> sNodes;  // pointer to 1st generation surface subdivision nodes
  ULONG l1stGenerationUCount ;
  ULONG l1stGenerationVCount ;

  // to prevent bad recursion back into BuildTree - turn off point testing
  SmTemporaryChangeValue<SmBoolean> sTCV1(m_bPointTestEnabled, FALSE);
  SmTemporaryChangeValue<SmBoolean> sTCV2(m_bFaceContainmentDone, 2);

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe  = FALSE;
SmBoolean bDebugMe1 = FALSE;
  if (bDebugMe) 
    {
      m_cpSurface->Dump();

      smgfx_Erase();
      smgfx_SetLook(1,2, 0,1,1) ; if(m_cpSurface) m_cpSurface->Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif

  // start m_pTree with a set of rl1stGenerationUCount X rl1stGenerationVCount subdivision 1st generation starter nodes 
  //      each with its own SmBezierPatch Data Object containing a gw_SURFACE object.
  //
  //   if(m_cpSurface is  SmBSPlineSurface) - Starter nodes = nonZero domain spans
  //   if(m_cpSurface not SmBsplineSurface) - Starter nodes = 5x5 (or less) grid of approximate BezierPatches
  //
  // Then each starter node is subdivided into children nodes until
  //     every child node's Bezier Patch passes the tolerance requirements.
  SmMemBlockMgr  sBezierBlock ;
  SmMemBlockMgr *pBezierBlock = pOptBezierBlock ? pOptBezierBlock : &sBezierBlock ;
  SER(BuildTreeBase(sNodes, l1stGenerationUCount, l1stGenerationVCount, pBezierBlock));

#ifdef SM_DEBUG_CODE
  if (bDebugMe) 
    {
      for(ULONG ii=0;ii<m_pTree->m_sNodeMgr.GetNumActiveElements();ii++)
        {
          SmTreeNode *pNode = (SmTreeNode*)m_pTree->m_sNodeMgr.GetAt(ii) ;
          pNode->Dump() ;
        }

      smgfx_Erase();
      smgfx_SetLook(1,2, 0,1,1) ; if(m_cpSurface) m_cpSurface->DrawUV(); sm_GraphicsLoop();
      smgfx_SetLook(1,2, 1,0,0) ; this->Draw() ;        sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // Add parent nodes to the starter nodes
  //   Each parent has just 2 children whose BezierPatch Domains span the parents Domain
  //    When pParentNode->m_pData->m_eSplitDir = 
  //      SM_SP_U: child1 = lower u domain half, left  side
  //               child2 = upper u domain half, right side
  //      SM_SP_V: child1 = lower v domain half, bot   side
  //               child2 = upper v domain half, top   side
  SER(BuildTreeTop(sNodes, l1stGenerationUCount, l1stGenerationVCount));

#ifdef SM_DEBUG_CODE
  if (bDebugMe) 
    {
      m_pTree->Dump() ;
      SmTArray<SmTreeNode*> sTreeNodes ;
      m_pTree->GetAllTreeNodes( sTreeNodes ) ;

      smgfx_Erase();
      smgfx_SetLook(1,2, 0,1,1) ; if(m_cpSurface) m_cpSurface->DrawUV(); sm_GraphicsLoop();
      smgfx_SetLook(1,2, 1,0,0) ; this->Draw(TRUE) ; sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // Clean up NURBS and Bezier pointers that become stale when m_sSubdisionSurface and sBezierBlock are freed
  if(pOptBezierBlock == NULL)
    {
      // need to 
      //   Free all m_sSubdivisionSurface memory.
      //   Set SmBezierAux2d::mBA_pSurface pointers to source surface.
      //   Set SmBezierPatch::m_pBezier pointers to NULL - they are construction only data.
        {
          SmObjsDelete<SmSurface*> sClean(&m_sSubdivisionSurfaces);

          ULONG ii ;
          ULONG lBezCount = m_sBezMgr.GetNumActiveElements() ;
          ULONG lAuxCount = m_sAuxMgr.GetNumActiveElements() ;
          for(ii=0;ii<lBezCount;ii++) { SmBezierPatch *pBezPatch = (SmBezierPatch *)m_sBezMgr.GetAt(ii) ;
                                        pBezPatch->mBA_pSurface  = m_cpSurface ;
                                        pBezPatch->m_pBezier     = NULL ;
                                      }
          for(ii=0;ii<lAuxCount;ii++) { SmBezierAux2d *pAux = (SmBezierAux2d *)m_sAuxMgr.GetAt(ii) ;
                                        pAux->mBA_pSurface  = m_cpSurface ;
                                      }
        }
      m_sSubdivisionSurfaces.ReSet();

    } // end need to clean up check

  // all done
#ifdef SM_DEBUG_CODE
  if (bDebugMe1) 
    {
      SM_ASSERT_VALID(m_pTree) ;
    }
  if (bDebugMe) 
    {
      m_pTree->Dump() ;
      SmTArray<SmTreeNode*> sTreeNodes ;
      m_pTree->GetAllTreeNodes( sTreeNodes ) ;

      smgfx_Erase();
      smgfx_SetLook(1,2, 0,1,1) ; if(m_cpSurface) m_cpSurface->DrawUV(); sm_GraphicsLoop();
      smgfx_SetLook(1,2, 1,0,0) ; this->Draw(TRUE) ; sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  return SM_SUCCESS;

} // end SmSurfaceCache::BuildTree

/*******************************************************************//**
PURPOSE: Get the Tree of the surface cache.  If it does not exist
   then build it.

NOTES: 
***********************************************************************/
SmTree * SmSurfaceCache::GetTree()
{
  const SmSurface*cpSurface = GetSurface() ;
  //std::lock_guard<std::recursive_mutex> lock(*(rSurface.mCacheMutex));

  if(   m_pTree == NULL 
     || (   this->m_bHaveTSurfaceCache == FALSE
         && cpSurface->GetOwner()
         && cpSurface->GetOwner()->IsKindOf(SmFace_TYPE)
         && this->m_bFaceContainmentDone != 2))
    {
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
      if(bDebugMe)
        {
          if(m_pTree) m_pTree->Dump() ;
        }
#endif

      if (BuildTree() != SM_SUCCESS) 
        {
          SE(SM_ERR);
          return NULL;
        }
    }
  return m_pTree;

} // end SmSurfaceCache::GetTree

/*******************************************************************//**
PURPOSE: Build the top of the tree.

NOTES: Add in ancester nodes over the 1st generation subdivision
  nodes crated in BuildTreeBase.  Each ancester node is of type SM_AD_AUX_DATA
  containing a SmBezierAux2d m_pData object.
***********************************************************************/
SmStatus SmSurfaceCache::BuildTreeTop
  (SmTArray<SmTreeNode*> & rNodes,      // in : array of 1st generation subdivision tree nodes
   ULONG &rl1stGenerationUCount,        // in : number of U entries in rNodes
   ULONG &rl1stGenerationVCount)        // in : number of V entries in rNodes
{
  // locals
  SmSurfaceCache * pSurfaceCache = this;
  SmTree         * pTree         = pSurfaceCache->m_pTree;
  ULONG            lNumU         = rl1stGenerationUCount;
  ULONG            lNumV         = rl1stGenerationVCount;

  // Now build up the top of the tree in some sort of reasonable way
  // We go along rows or columns and combine nodes

  // init aux memory manager
  ULONG lNumPerBlock = lNumU*lNumV + 10;
  if (lNumPerBlock > 100) lNumPerBlock = 100;
  if (lNumPerBlock < 30)  lNumPerBlock = 30;
  m_sAuxMgr.Initialize(ALIGN_SIZE(sizeof(SmBezierAux2d)),lNumPerBlock) ;
  
  // make current and next generation node lists
  SmTArray<SmTreeNode*> sNodes(rNodes.GetSize());
  sNodes.Append(rNodes);
  SmTArray<SmTreeNode*> sNodes2(sNodes.GetSize());
  SmTArray<SmTreeNode*> *pCurr = &sNodes;
  SmTArray<SmTreeNode*> *pNext = &sNodes2;

  // while current generation has more than 1 node
  while (lNumU > 1 || lNumV > 1) 
    {
      SmSurfParamType sCombineDirection = SM_SP_U;
      if (lNumU == 1) 
        {
          sCombineDirection = SM_SP_V;
        }
      else if (lNumV != 1) // more than 1 U and more than 1 V nodes
        {
          // Look at combined bounding box sizes to see which direction would be 
          // the best to combine (the shortest).  Note that this will only give a guess and 
          // the other portions of the grid may give other answers.  But this
          // is ok - we are just making a tree.
          SmTreeNode *pNode00 = (SmTreeNode*)(*pCurr)[0];
          SmTreeNode *pNode01 = (SmTreeNode*)(*pCurr)[1];
          SmTreeNode *pNode10 = (SmTreeNode*)(*pCurr)[lNumV];
          SmExtent3d sExtU, sExtV;
          pNode00->m_sBBox.Union(pNode01->m_sBBox,sExtV);
          pNode00->m_sBBox.Union(pNode10->m_sBBox,sExtU);

          SM_ASSERT_MSG(pNode00->m_sBBox.IsContainedBy(sExtV, SM_EFF_ZERO_SQRT),_T("pChild1 BBox not contained in parent BBox")) ;
          SM_ASSERT_MSG(pNode01->m_sBBox.IsContainedBy(sExtV, SM_EFF_ZERO_SQRT),_T("pChild1 BBox not contained in parent BBox")) ;

          SM_ASSERT_MSG(pNode00->m_sBBox.IsContainedBy(sExtU, SM_EFF_ZERO_SQRT),_T("pChild1 BBox not contained in parent BBox")) ;
          SM_ASSERT_MSG(pNode10->m_sBBox.IsContainedBy(sExtU, SM_EFF_ZERO_SQRT),_T("pChild1 BBox not contained in parent BBox")) ;

          SmVector3d sDiffU = sExtU.GetMax() - sExtU.GetMin();
          SmVector3d sDiffV = sExtV.GetMax() - sExtV.GetMin();
          if (sDiffV.LengthSquared() < sDiffU.LengthSquared()) 
            {
              sCombineDirection = SM_SP_V;
            }
        }

      // Now we have to do something different depending upon which 
      // direction we are combining.  Note that the array contains
      // U ordered patches.  
      // Note that patches are ordered in such a way that the nodes with
      // a constant V are sequential.
      if (sCombineDirection == SM_SP_V) 
        {  // Combine along rows of V
          // This combination shrinks the rows
          for (ULONG ii=0; ii<lNumU; ii++) 
            {
              for (ULONG kk=1; kk<lNumV; kk=kk+2) 
                {
                  SmTreeNode    *pNode1         = (SmTreeNode*)(*pCurr)[ii*lNumV+kk-1];
                  SmTreeNode    *pNode2         = (SmTreeNode*)(*pCurr)[ii*lNumV+kk];
                  SmTreeNode    *pNewNode       = (SmTreeNode*)pTree->m_sNodeMgr.GetNewElement();
                  SmBezierAux2d *pAux           = (SmBezierAux2d*)pSurfaceCache->m_sAuxMgr.GetNewElement();
                  SmBezierAux2d *pAux1          = (SmBezierAux2d*)pNode1->m_pData;
                  SmBezierAux2d *pAux2          = (SmBezierAux2d*)pNode2->m_pData;
                  pNext->Add(pNewNode);         
                                                
                  pNewNode->m_pTree             = pTree;
                  pNewNode->m_pParent           = NULL;
                  pNewNode->m_pChild1           = pNode1;
                  pNewNode->m_pChild2           = pNode2;
                  pNode1->m_sBBox.Union(pNode2->m_sBBox,pNewNode->m_sBBox);
                  SM_ASSERT_MSG(pNode1->m_sBBox.IsContainedBy(pNewNode->m_sBBox, SM_EFF_ZERO_SQRT),_T("pChild1 BBox not contained in parent BBox")) ;
                  SM_ASSERT_MSG(pNode2->m_sBBox.IsContainedBy(pNewNode->m_sBBox, SM_EFF_ZERO_SQRT),_T("pChild1 BBox not contained in parent BBox")) ;

                  pNewNode->m_eAuxDataType      = SM_AD_BEZIER_SURFACE_HEAD;
                  pNewNode->m_pData             = pAux;
                  pNewNode->m_eGeomType         = SM_NG_DEFAULT;
                                                
                  pNode1->m_pParent             = pNewNode;
                  pNode2->m_pParent             = pNewNode;
                                                
                  pAux->m_eSplitDir             = SM_SP_V;
                  pAux->m_bIsMarked             = FALSE;
                  pAux->m_bIsTessOnly           = FALSE;
                  pAux->m_eNodeClass            = SM_NC_UNKNOWN;
                  pAux->m_pFace                 = NULL;
                  pAux->m_pOwningTreeNode       = pNewNode;
                  pAux1->m_sUVDomain.Union(pAux2->m_sUVDomain,pAux->m_sUVDomain);
                  //pAux1->m_sPolarBox.Union(pAux2->m_sPolarBox, pAux->m_sPolarBox) ;
                      
                  pAux->m_vVertexList.Init();
                  pAux->m_sEdgeuseList.Init();
                  pAux->m_sPolyEdgeList.Init();
                  pAux->mBA_pSurface            = NULL;
                  pAux->m_bChordHeightSatisfied = FALSE;
                  pAux->m_bAngleTolSatisfied    = FALSE;
// GWCTreeVertexTemp
//                  pAux->m_lStartTreeVertexIndx  = SM_BIG_ULONG ; // internal nodes don't have TreeVertex loops
                }
              if (lNumV % 2 == 1) 
                {
                  pNext->Add((*pCurr)[ii*lNumV+lNumV-1]);  // Add on last element if had odd number
                }
            }
          lNumV = (lNumV+1) / 2;  // Used up either half or one less columns 
        }
      else 
        { // Combine in V direction
          for (ULONG ii=1; ii<lNumU; ii=ii+2) 
            {
              for (ULONG kk=0; kk<lNumV; kk++) 
                {
                  SmTreeNode    *pNode1         = (SmTreeNode*)(*pCurr)[(ii-1)*lNumV+kk];
                  SmTreeNode    *pNode2         = (SmTreeNode*)(*pCurr)[ii*lNumV+kk];
                  SmTreeNode    *pNewNode       = (SmTreeNode*)pTree->m_sNodeMgr.GetNewElement();
                  SmBezierAux2d *pAux           = (SmBezierAux2d*)pSurfaceCache->m_sAuxMgr.GetNewElement();
                  SmBezierAux2d *pAux1          = (SmBezierAux2d*)pNode1->m_pData;
                  SmBezierAux2d *pAux2          = (SmBezierAux2d*)pNode2->m_pData;
                  pNext->Add(pNewNode);         
                                                
                  pNewNode->m_pTree             = pTree;
                  pNewNode->m_pParent           = NULL;
                  pNewNode->m_pChild1           = pNode1;
                  pNewNode->m_pChild2           = pNode2;
                  pNode1->m_sBBox.Union(pNode2->m_sBBox,pNewNode->m_sBBox);
                  SM_ASSERT_MSG(pNode1->m_sBBox.IsContainedBy(pNewNode->m_sBBox, SM_EFF_ZERO_SQRT),_T("pChild1 BBox not contained in parent BBox")) ;
                  SM_ASSERT_MSG(pNode2->m_sBBox.IsContainedBy(pNewNode->m_sBBox, SM_EFF_ZERO_SQRT),_T("pChild1 BBox not contained in parent BBox")) ;

                  pNewNode->m_eAuxDataType      = SM_AD_BEZIER_SURFACE_HEAD;
                  pNewNode->m_pData             = pAux;
                  pNewNode->m_eGeomType         = SM_NG_DEFAULT;

                  pNode1->m_pParent             = pNewNode;
                  pNode2->m_pParent             = pNewNode;

                  pAux->m_eSplitDir             = SM_SP_U;
                  pAux->m_bIsMarked             = FALSE;
                  pAux->m_bIsTessOnly           = FALSE;
                  pAux->m_eNodeClass            = SM_NC_UNKNOWN;
                  pAux->m_pFace                 = NULL;
                  pAux->m_pOwningTreeNode       = pNewNode;
                  pAux1->m_sUVDomain.Union(pAux2->m_sUVDomain,pAux->m_sUVDomain);
                  //pAux1->m_sPolarBox.Union(pAux2->m_sPolarBox, pAux->m_sPolarBox) ;
                  pAux->m_vVertexList.Init();
                  pAux->m_sEdgeuseList.Init();
                  pAux->m_sPolyEdgeList.Init();
                  pAux->mBA_pSurface            = NULL;
                  pAux->m_bChordHeightSatisfied = FALSE;
                  pAux->m_bAngleTolSatisfied    = FALSE;
// GWCTreeVertexTemp
//                  pAux->m_lStartTreeVertexIndx  = SM_BIG_ULONG ; // internal nodes don't have TreeVertex loops
                }
            }
          // If have odd number of rows add on last one
          if (lNumU % 2 == 1) 
            {
              for (ULONG kk=0; kk<lNumV; kk++) 
                {
                  ULONG lIndex = (lNumU-1)*lNumV + kk;
                  pNext->Add((*pCurr)[lIndex]);
                }
            }
          lNumU = (lNumU+1) / 2;  // Used up either half or one less rows
        }

      // Swap arrays
      SmTArray<SmTreeNode*> *pTmp = pCurr;
      pCurr = pNext;
      pNext = pTmp;
      pNext->ReSet();
    }     // While loop

  pTree->m_pTopNode = (SmTreeNode*)(*pCurr)[0];
  //      if(pTree->m_pTopNode->m_eAuxDataType == SM_AD_AUX_DATA)
  //        { pTree->m_pTopNode->m_eAuxDataType = SM_AD_BEZIER_SURFACE_HEAD; }
  return SM_SUCCESS;

} // end SmSurfaceCache::BuildTreeTop

/*******************************************************************//**
PURPOSE: Constructor for SmSurfaceCache.  It allows input of data used
    during tessellation of the cache.  Note that this constructor only
    inputs values.  The actual creation of cache data is done in the 
    Tessellate method.  

NOTES: All arguments except the curve and interval are optional.
***********************************************************************/
SmSurfaceCache::SmSurfaceCache
  (const SmSurface  & crSurface,
   const SmExtent2d & crUVDomain,
   double             dChordHeightTolerance,
   double             dAngTolRad,
   double             dAspectRatio3D,
   double             dMaxSideLength3D,
   double             dMinSideLength3D,
   double             dMinSideLengthRatioUV)                               
:  m_lNumAltCaches         (0),

   m_pTree         (NULL),
   m_cpSurface     (&crSurface),
   m_sUVDomain     (crUVDomain),
   m_pUContinuities(NULL),
   m_pVContinuities(NULL),

   m_lBezierSize  (0),
   m_lTotalBezSize(0),
   //      m_lNumUSpans   (0),   
   //      m_lNumVSpans   (0),  

   m_dChordHeightTolerance      (dChordHeightTolerance),
   m_dAngTolRad                 (dAngTolRad),        
   m_dAspectRatio3D             (dAspectRatio3D),
   m_dMaximumSideLength3D       (dMaxSideLength3D),
   m_dMinimumSideLength3D       (dMinSideLength3D),
   m_dMinimumSideLengthRatioUV  (dMinSideLengthRatioUV),
   m_dIsoCurveTolerance         (.001), 

   m_eTessellationAlgorithm(SM_TA_FASTER_TESSELLATION),
   m_dStartFastSubdivisionAngleDeg(30.0),
   m_bSwitchToFastSubdivision     (FALSE),

   m_dTrimSubdivisionFactor       (20.0),
   m_sBoundingPlanes              (&crSurface, crUVDomain),

   m_bHaveTSurfaceCache    (FALSE),
   m_bFaceWasModified      (FALSE),
   m_bProcessBoundaryCurves(TRUE),

   m_bPointTestEnabled   (FALSE),
   m_bFaceContainmentDone(FALSE) 
{
  // count caches made and inform the debugging public
#ifdef SM_DEBUG_CACHE_CPP
static ULONG lCount = 0 ; lCount++ ;
  TCHAR sBuff[SM_TBLOCK_SIZE] ;

  ((SmSurface *)m_crSurface)->m_lSurfaceCacheCount++ ;
  smos_sprintf(sBuff, _T("\nConstructed Surface Cache [Surf = 0x%p, rep = %d] iter: %d"), 
                    m_crSurface, 
                    m_cpSurface->m_lSurfaceCacheCount, 
                    lCount) ;
  MYPRINTF(sBuff) ;
#endif

  // compute memory size in bytes of one bezier patch with this surface's degrees
  m_lBezierSize =   crSurface.IsKindOf(SmBSplineSurface_TYPE)
                  ? sm_ComputeNurbSurfaceSize(crSurface.GetDegree(SM_SP_U),
                                                 crSurface.GetDegree(SM_SP_V), 
                                                 crSurface.GetDegree(SM_SP_U)*2+1, 
                                                 crSurface.GetDegree(SM_SP_V)*2+1)
                  : sm_ComputeNurbSurfaceSize(3,
                                                 3, 
                                                 3*2+1, 
                                                 3*2+1) ;

  // Memory for SmBezierPatch object is augmented to hold the Bezier Patch data
  //   in one contiguous block.  The SmBezierPatch class is declared to hold
  //   only the top level pointers of the patch.
  m_lTotalBezSize = ALIGN_SIZE(sizeof(SmBezierPatch)) ;

  // init internal array values
  m_pUContinuities = new (*GetContext()) SmTArray<SmContinuityType>(*GetContext());
  m_pVContinuities = new (*GetContext()) SmTArray<SmContinuityType>(*GetContext());
  m_apIsoBoundaryCurves[0] = NULL;
  m_apIsoBoundaryCurves[1] = NULL;
  m_apIsoBoundaryCurves[2] = NULL;
  m_apIsoBoundaryCurves[3] = NULL;
  for (ULONG i=0; i<SM_MAX_ALT_CACHES; i++) 
    {
      m_apAltSrfCaches[i] = NULL;
    }

} // end SmSurfaceCache::SmSurfaceCache constructor

/*******************************************************************//**
PURPOSE: Set the trim subdivision factor for the surface cache.  It 
    used to control how much subdivision is allowed for adding trim curves.

NOTES: Value is bounded between 10 and 100.
***********************************************************************/
void SmSurfaceCache::SetTrimSubdivisionFactor
  (double dSubdivisionFactor)
{
  m_dTrimSubdivisionFactor =    dSubdivisionFactor > 100.0 ? 100.0
                              : dSubdivisionFactor <  10.0 ?  10.0
                              : dSubdivisionFactor ;

} // end SmSurfaceCache::SetTrimSubdivisionFactor

/*******************************************************************//**
PURPOSE: Get or create the Iso Boundary curves given a UV Domain of 
    a surface.  This method may create new curves or give curves stored
    in the surface cache.  This method's primary purpose in life is to
    speed up some of the operations that use the boundary curves.  It is
    not a user routine.  The curves will always be created in the context
    of the surface cache.  This should not only save the creation of the
    ISO curves but also in creating their caches.

NOTES: This method creates and stores even degenerate boundary
    curves.  If the curves are not cached then the user is responsible
    for destroying them.
***********************************************************************/
SmStatus SmSurfaceCache::GetIsoBoundaryCurves
  (const SmExtent2d & crUVDomain,           // in : target Surface UVDomain
   double             dApproxTol3d,         // in : max dist between returned isoParameter Curves and Surface
   SmBoolean & rbCurvesAreCached,           // out: TRUE = returned curves will be deleted with surface cache
                                            //      FALSE= user should delete output curves after done using them.  
   SmTArray<SmBSplineCurve*> & rIsoCurves)  // out: IsoParameter Curves bounding input crUVDomain
                                            //      ordered: [0] - minimum const U curve   //       3     
                                            //               [1] - minimum const V curve   //       |     
                                            //               [2] - maximum const U curve   //     +----+  
                                            //               [3] - maximum const V curve   //     |    |  
{                                           //                                             //   0-|    |-2
  // init output                            //                                             //     +----+  
  rbCurvesAreCached = FALSE;                //                                             //       |     
  rIsoCurves.ReSet();                       //                                             //       1     

  // See if crUVDomain matches the domain of the cache
  SmVector2d sDiffMin    = crUVDomain.GetMin() - m_sUVDomain.GetMin();
  SmVector2d sDiffMax    = crUVDomain.GetMax() - m_sUVDomain.GetMax();
  SmBoolean  bSameDomain =   (   sDiffMin.LengthSquared() > SM_EFF_ZERO_SQ       
                              || sDiffMax.LengthSquared() > SM_EFF_ZERO_SQ       
                              || dApproxTol3d             < m_dIsoCurveTolerance)
                           ? FALSE
                           : TRUE ;

  // If the domains match and there are cached boundary curves use them
  if (   bSameDomain 
      && m_apIsoBoundaryCurves[0] != NULL) 
    {
      rbCurvesAreCached = TRUE;
      rIsoCurves.Add(m_apIsoBoundaryCurves[0]);
      rIsoCurves.Add(m_apIsoBoundaryCurves[1]);
      rIsoCurves.Add(m_apIsoBoundaryCurves[2]);
      rIsoCurves.Add(m_apIsoBoundaryCurves[3]);
      return SM_SUCCESS;
    }

  // arrive here when we need to make isoParameter boundary curves

  // remember the ApproxTol3d used to Create these IsoParamCurves
  m_dIsoCurveTolerance = dApproxTol3d;

  // build isoParameter curves
  SmBSplineCurve *pNewBSC = NULL ;
  SER(m_cpSurface->CreateIsoParametricCurve(*m_cpSurface->GetContext(), 
                                            SM_SP_U,
                                            crUVDomain.GetMin().x,
                                            dApproxTol3d,
                                            pNewBSC,
                                            &crUVDomain));
  rIsoCurves.Add(pNewBSC); pNewBSC = NULL ;
  SER(m_cpSurface->CreateIsoParametricCurve(*m_cpSurface->GetContext(), 
                                            SM_SP_V,
                                            crUVDomain.GetMin().y,
                                            dApproxTol3d,
                                            pNewBSC,
                                            &crUVDomain));
  rIsoCurves.Add(pNewBSC); pNewBSC = NULL ;
  SER(m_cpSurface->CreateIsoParametricCurve(*m_cpSurface->GetContext(), 
                                            SM_SP_U,
                                            crUVDomain.GetMax().x,
                                            dApproxTol3d,
                                            pNewBSC,
                                            &crUVDomain));
  rIsoCurves.Add(pNewBSC); pNewBSC = NULL ;
  SER(m_cpSurface->CreateIsoParametricCurve(*m_cpSurface->GetContext(), 
                                            SM_SP_V,
                                            crUVDomain.GetMax().y,
                                            dApproxTol3d,
                                            pNewBSC,
                                            &crUVDomain));
  rIsoCurves.Add(pNewBSC); pNewBSC = NULL ;

  // when domain is same as cache domain - cache curves
  if (bSameDomain) 
    {
      rbCurvesAreCached = TRUE;
      m_apIsoBoundaryCurves[0] = rIsoCurves[0];
      m_apIsoBoundaryCurves[1] = rIsoCurves[1];
      m_apIsoBoundaryCurves[2] = rIsoCurves[2];
      m_apIsoBoundaryCurves[3] = rIsoCurves[3];
    }

  return SM_SUCCESS;

} // end SmSurfaceCache::GetIsoBoundaryCurves

/*******************************************************************//**
PURPOSE: Destructor for the surface cache.

NOTES: Assumes that even degenerate curves are created and
    stored in the array.
***********************************************************************/
SmSurfaceCache::~SmSurfaceCache()
{
#ifdef SM_DEBUG_CACHE_CPP
static ULONG lCount = 0 ; lCount++ ;
  TCHAR sBuff[SM_TBLOCK_SIZE] ;
  smos_sprintf(sBuff,_T("\nDestructed Surface Cache [Surface = 0x%p] iter: %ld"),
          m_cpSurface, lCount) ;
  MYPRINTF(sBuff) ;
#endif

  if (m_pTree)          { delete m_pTree;          m_pTree          = NULL ; }
  if (m_pUContinuities) { delete m_pUContinuities; m_pUContinuities = NULL ; }
  if (m_pVContinuities) { delete m_pVContinuities; m_pVContinuities = NULL ; }
  if (m_cpSurface)      { m_cpSurface = NULL ; }
  if (m_apIsoBoundaryCurves[0]) 
    {
      SM_ASSERT(m_apIsoBoundaryCurves[0] != NULL) ; delete m_apIsoBoundaryCurves[0] ; m_apIsoBoundaryCurves[0] = NULL ;
      SM_ASSERT(m_apIsoBoundaryCurves[1] != NULL) ; delete m_apIsoBoundaryCurves[1] ; m_apIsoBoundaryCurves[1] = NULL ;
      SM_ASSERT(m_apIsoBoundaryCurves[2] != NULL) ; delete m_apIsoBoundaryCurves[2] ; m_apIsoBoundaryCurves[2] = NULL ;
      SM_ASSERT(m_apIsoBoundaryCurves[3] != NULL) ; delete m_apIsoBoundaryCurves[3] ; m_apIsoBoundaryCurves[3] = NULL ;
    }

  for (ULONG i=0; i<m_lNumAltCaches; i++) 
    { SM_ASSERT(m_apAltSrfCaches[i] != NULL) ; delete m_apAltSrfCaches[i] ; m_apAltSrfCaches[i] = NULL ; }

  {
      SmObjsDelete<SmSurface*> sClean(&m_sSubdivisionSurfaces);
  }

} // end SmSurfaceCache::~SmSurfaceCache destructor

/*******************************************************************//**
PURPOSE: Return Brep Pointer when associated surface is part of a
            Brep, else return NULL.

NOTES: 
***********************************************************************/
SmBrep * SmSurfaceCache::GetBrep() 
 const
{
  SmObject *pOwner = m_cpSurface->GetOwner() ;
  if(pOwner)
    {
      // switch on Owner type to return owning Brep
      if(pOwner->GetType() == SmFace_TYPE)
        {
          return( ((SmFace *)pOwner)->GetBrep() ) ;
        }
// Remove Composite
//      else if(pOwner->GetType() == SmCFace_TYPE)
//        {
//          return( ((SmCFace *)pOwner)->GetBrep() ) ;
//        }

    } // end surface has owner check

  // arrive here when surface has no owner
  return(NULL) ;

} // end SmSurfaceCache::GetBrep

/*******************************************************************//**
PURPOSE: Simple method to get the face of a Timmed surface cache.
    Note that the face may be a composite.

NOTES: 
***********************************************************************/
SmFace * SmSurfaceCache::GetFace
  () 
 const
{  
  return (SmFace*)m_cpSurface->GetFace();
} // end SmSurfaceCache::GetFace



/*******************************************************************//**
PURPOSE: Compute the total size of the memory used by the surface cache.

NOTES: 
***********************************************************************/
ULONG SmSurfaceCache::GetMemoryUsed
  (ULONG &rlMemoryAllocated) 
 const
{
   ULONG lBezAlloc=0, lAuxAlloc=0, lVLAlloc=0, lEdgeuseListAlloc=0, lPolyEdgeListAlloc =0, lTreeAlloc=0 ;
   ULONG lUContinuitiesAlloc=0, lVContinuitiesAlloc=0;

   ULONG lUsed =   sizeof(this) 
                 + m_sBezMgr.GetMemoryUsed(lBezAlloc) 
                 + m_sAuxMgr.GetMemoryUsed(lAuxAlloc) 
                 + m_sVLMgr.GetMemoryUsed(lVLAlloc) 
                 + m_sEdgeuseListMgr.GetMemoryUsed(lEdgeuseListAlloc)
                 + m_sPolyEdgeListMgr.GetMemoryUsed(lPolyEdgeListAlloc);

   ULONG lTreeUsed          = m_pTree          ? m_pTree->GetMemoryUsed(lTreeAlloc)                   : 0 ;
   ULONG lUContinuitiesUsed = m_pUContinuities ? m_pUContinuities->GetMemoryUsed(lUContinuitiesAlloc) : 0 ;
   ULONG lVContinuitiesUsed = m_pVContinuities ? m_pVContinuities->GetMemoryUsed(lVContinuitiesAlloc) : 0 ;

   // todo: something for m_apAltSrfCaches array
   //  put it has been published with a void * pointer to hold user data. 
   //  No way to size that.

   // todo: something for m_apIsoBoudnaryCurves

   // set output
   rlMemoryAllocated =   sizeof(this)
                       + lBezAlloc 
                       + lAuxAlloc 
                       + lVLAlloc 
                       + lEdgeuseListAlloc
                       + lPolyEdgeListAlloc
                       + lTreeAlloc ; 

   // all done
   return lUsed + lTreeUsed + lUContinuitiesUsed + lVContinuitiesUsed;

} // end SmSurfaceCache::GetMemoryUsed

/*******************************************************************//**
 PURPOSE: Judge whether 2DPoint is in/on/out of this surface->Owner's
 TrimBoundaries.

 NOTES:
 A SurfaceCache has no trim boundaries so this function always returns TRUE.
 The Derived TrimSrfCache has trim boundaries and makes a more
 complicated check.
 ***********************************************************************/
SmStatus SmSurfaceCache::PointTest
( 
  const SmPoint2d &crUVPoint,    // in : TargetPoint
  SmBoolean & rbPointIsOk,       // out: TRUE = point is passes this TrimBoundary Check 
  SmZoneTol3d *,                 // in : p3dTolerance = max dist for counting a near miss a hit
  SmObject **                    // out: Optional Pointer to Topology Object coincident with point,
                                 //      NULL to ignore, default:[NULL]
) const
{
  // skip point testing when asked
  if(!m_bPointTestEnabled)
  {
    rbPointIsOk = TRUE;   // return true so this point won't be culled
  }
  else // return FALSE = outside face, TRUE = in/on face
  {
    rbPointIsOk = m_sUVDomain.ContainsPoint2d( crUVPoint );
  }

  // all done
  return SM_SUCCESS;

} // end SmSurfaceCache::PointTest

/*******************************************************************//**
PURPOSE:

NOTES: This base function should never be called. Just its
derived virtual versions
***********************************************************************/
SmStatus SmSurfaceCache::PointClassify
( 
  const SmPoint2d            &,  // in : Point to classify                   
  SmZoneTol3d,                   // in : Obj ZoneTol3d assoc with UVPoint, not this face 
                                 //     (if none, use: SmTol::GetZoneTol3d(Brep_Context_Or_NULL)
  SmPointClassificationType &,   // out: Type of object coincident with point
                                 //      oneof SM_PC_VERTEX                  
                                 //            SM_PC_EDGE                    
                                 //            SM_PC_FACE                    
                                 //            SM_PC_UNKNOWN                 
  SmObject *&                    // out: object coincident with point        
 ) const
{
  SE( SM_ERR );
  return SM_ERR;

} // end SmSurfaceCache::PointClassify

/*******************************************************************//**
PURPOSE: AssertValid() Test reporting static labels
***********************************************************************/
SmAssertReportLabel sAssertSurfaceCache_list[] =
{
 /*  0 */ {SM_AT_SIZE, _T("Bad InsideTreeNode"),     _T("INSIDE classified SurfaceCacheNode has NonEmpty Vertex or Curve list. Should be empty.") },
 /*  1 */ {SM_AT_SIZE, _T("Bad OutsideTreeNode"),    _T("OUTSIDE classified SurfaceCacheNode has NonEmpty Vertex or Curve list. Should be empty.") },
 /*  2 */ {SM_AT_SIZE, _T("Bad BoundaryTreeNode"),   _T("BOUNDARY classified SurfaceCacheNode has empty Vertex or Curve list. Should be NonEmpty.") }, 
 /*  3 */ {SM_AT_FLAG, _T("Bad TreeNode"),           _T("Node remains unclassified (in/on/out) after FaceContainment is Done") },
 /*  4 */ {SM_AT_SIZE, _T("Bad Assoc Arrays sizes"), _T("m_pUContinuities list size not equal to Surface Unique u Knot list size") },
 /*  5 */ {SM_AT_SIZE, _T("Bad Assoc Arrays sizes"), _T("m_pVContinuities list size not equal to Surface Unique u Knot list size") }
} ;

/*******************************************************************//**
PURPOSE: Check the validity of an SmSurfaceCache.

NOTES: Checks made:
 - Once m_bFaceContainmentDone == TRUE
    - check every leaf node to see that
       pNode->pAux->m_eNodeClass == SM_NC_INSIDE   and m_vVertexList, m_sEdgeuseList, and m_sPolyEdgeList lengths are zero
       pNode->pAux->m_eNodeClass == SM_NC_OUTSIDE  and m_vVertexList, m_sEdgeuseList, and m_sPolyEdgeList lengths are zero
       pNode->pAux->m_eNodeClass == SM_NC_BOUNDARY and m_vVertexList or m_sEdgeuseList or m_sPolyEdgeList lengths are nonzero

RETURNS ---  TRUE  = OK
             FALSE = Problem
***********************************************************************/
SmBoolean SmSurfaceCache::AssertValid
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

  // check base class validity                     
  //      SmBoolean bRtn = SmCacheObj::AssertValid(pAList, eTestLevel, FALSE, pTestRequests) ;  
  bRtn &= (  (eTestLevel != SM_LEVEL_GIVEN)
           ? SmObject::AssertValid(pAList, eTestLevel, SM_NO_WALK, pTestRequests) 
           : TRUE ) ; 

  // low work - no subdivision tree yet to check
  if(m_pTree == NULL)
    {
      return(bRtn) ; 
    } 

  // check the tree
  bRtn &= m_pTree->AssertValid(pAList) ;

  // locals
  ULONG ii ;
  SmBoolean bOK = TRUE ;

#ifdef SM_DEBUG_CODE
  const SmSurface *cpSurface = GetSurface() ;
#endif

  SmTArray<SmTreeNode*> sTreeNodes ;
  m_pTree->GetAllTreeNodes(sTreeNodes) ;

  // Sanity check - all leaf nodes which are on the boundary should have some
  // vertices and/or edges in their lists
  for(ii=0;ii<sTreeNodes.GetSize();ii++)
    {
      SmTreeNode    *pNode = sTreeNodes[ii] ;
      SmBezierAux2d *pAux  = (SmBezierAux2d*)pNode->m_pData ;

      // skip nonLeaf nodes
      if(pNode->m_pChild1)
        { continue ; }

      // get the node;s vertelists and curvelists
      SmTArray<SmVertexList*> sVertexLists ;
      SmTArray<SmEdgeuseList*>  sEdgeuseLists ;
      SmTArray<SmPolyEdgeList*> sPolyEdgeLists;
      
      pAux->m_vVertexList.GetAllNodes(sVertexLists) ;
      pAux->m_sEdgeuseList.GetAllNodes(sEdgeuseLists) ;
      pAux->m_sPolyEdgeList.GetAllNodes(sPolyEdgeLists);

      // in classified tree nodes should have no vertices or edges
      if(pAux->m_eNodeClass == SM_NC_INSIDE)           { 
          bOK = (   sVertexLists.GetSize() == 0 && sEdgeuseLists.GetSize()  == 0 && sPolyEdgeLists.GetSize()) ;
          bRtn &= SM_ASSERT_BOOLEAN_REPORT(0, SM_LEVEL_0, (bOK), _T("") ) ;
      }
      else if(pAux->m_eNodeClass == SM_NC_OUTSIDE)     { 
          bOK = (sVertexLists.GetSize() == 0 && sEdgeuseLists.GetSize() == 0 && sPolyEdgeLists.GetSize());
          bRtn &= SM_ASSERT_BOOLEAN_REPORT(1, SM_LEVEL_0, (bOK), _T("") ) ;
      }
      else if(pAux->m_eNodeClass == SM_NC_ON_BOUNDARY) { 
          bOK = (   sVertexLists.GetSize() != 0 || sEdgeuseLists.GetSize()  != 0 || sPolyEdgeLists.GetSize() != 0) ;
          bRtn &= SM_ASSERT_BOOLEAN_REPORT(2, SM_LEVEL_0, (bOK), _T("") ) ;
      }
      else                                             { 
          bRtn &= SM_ASSERT_BOOLEAN_REPORT(3, SM_LEVEL_0, (m_bFaceContainmentDone != TRUE), _T("") ) ;
          // note: valid values here are currently FALSE and '2' 
      }

    } // end iter every TreeNode checking containment state

  // surface unique knot count should equal the m_pUContinuity and m_pVContinuity array sizes
  SmTArray<double> sUKnots, sVKnots ;
  m_cpSurface->GetKnots(SM_SP_U, sUKnots ) ; // arrays are not computed in SmTessSrfCache::BuildTreeWithSubdivision
  m_cpSurface->GetKnots(SM_SP_V, sVKnots ) ; // because the continuities are not needed for tessellations.  Allow for array lengths to be zero
  bOK = m_pUContinuities->GetSize() == 0 || m_pUContinuities->GetSize() == sUKnots.GetSize() ; 
  bRtn &= SM_ASSERT_BOOLEAN_REPORT(4, SM_LEVEL_0, (bOK), _T("") ) ;
  bOK = m_pVContinuities->GetSize() == 0 || m_pVContinuities->GetSize() == sVKnots.GetSize() ; 
  bRtn &= SM_ASSERT_BOOLEAN_REPORT(5, SM_LEVEL_0, (bOK), _T("") ) ;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  // draw 
  if(bDebugMe)
    {
      SmFace *pFace = (SmFace *)cpSurface->GetFace();

      Dump() ;

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,1,0) ; if(cpSurface) cpSurface->DrawUV() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,0) ; if(pFace) pFace->Draw(SM_DM_CROSSHATCH, 6,6) ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 0,1,1) ; Draw() ; sm_GraphicsLoop() ;                      // don't draw BBoxes
      smgfx_SetLook(3,4, 0,1,1) ; Draw(TRUE) ; sm_GraphicsLoop() ;                  // draw BBoxes
      smgfx_SetLook(3,4, 0,1,1) ; Draw(TRUE,FALSE,TRUE,TRUE) ; sm_GraphicsLoop() ;  // draw just BBoxes on boundaries
      smgfx_SetLook(3,4, 0,1,1) ; Draw(TRUE,FALSE,TRUE,FALSE) ; sm_GraphicsLoop() ; // draw just BBoxes with vertices
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // all done 
  // SM_ASSERT( bRtn );

  return( bRtn );

} // end SmSurfaceCache::AssertValid
    
// obsolete
// /*******************************************************************//**
// PURPOSE: Fix Assert Report failures reported by an AssertValid call
// 
// RETURNS ---  TRUE  = OK
//              FALSE = Problem
// ***********************************************************************/
// SmBoolean SmSurfaceCache::AssertHeal
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
//       return ( SmCacheObj::AssertHeal(rAReport, pAList) ) ;
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
//                rAReport.m_pHealMessage = _T("SmSurfaceCache::AssertHeal fix not yet supported") ;  
//                 
//     }
// 
//   // all done
//   return(bRtn) ;
// 
// } // end SmSurfaceCache::AssertHeal
// end obsolete

/*******************************************************************//**
PURPOSE:
NOTES:
***********************************************************************/
SmBoolean SmSurfaceCache::IsKindOf( SM_TYPE t ) const
{
  return ((SmSurfaceCache_TYPE == t) ? TRUE : SmObject::IsKindOf( (t) ));
}


/********************************************************************
PURPOSE:

NOTES:
*********************************************************************/
void SmSurfaceCache::Dump
  (void) 
 const
{
  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE];

  ULONG ii ;
  SmTArray<void*> sNodes;
  m_pTree->m_sNodeMgr.GetActiveElements(sNodes);

  smos_WriteBuffer(_T("\nBegin SmSurfaceCache Dump()"));

  // this label
  smos_sprintf(sBuff,       _T("\nSmSurfaceCache = 0x%p, Number of Nodes = %ld, NumAltCaches = %ld"),this,sNodes.GetSize(), m_lNumAltCaches);
  smos_sprintf(sBuffForFile,_T("\nSmSurfaceCache = %s, Number of Nodes = %ld, NumAltCaches = %ld"),_T("notNULL"),sNodes.GetSize(), m_lNumAltCaches);
  smos_WriteBuffer(sBuff, sBuffForFile);

  // m_cpSurface
  smos_sprintf(sBuff,       _T("\n  Surface[%s] = 0x%p"), m_cpSurface->GetClassString(), m_cpSurface) ;
  smos_sprintf(sBuffForFile,_T("\n  Surface[%s] = noNULL"), m_cpSurface->GetClassString()) ;
  smos_WriteBuffer(sBuff, sBuffForFile);

  // UVDomain
  smos_WriteBuffer(_T("\n  UVDomain = ")); m_sUVDomain.Dump() ;  

  // Tessellation Control                                                        
  smos_sprintf(sBuff, _T("\n  ChordHeightTol      : %16.16lf\n  AngTolRad           : %16.16lf\n  AspectRatio3D       : %16.16lf"),
             m_dChordHeightTolerance,   
             m_dAngTolRad,         
             m_dAspectRatio3D) ;          
  smos_WriteBuffer(sBuff) ;
                                                          
  smos_sprintf(sBuff, _T("\n  MaxSizeLength3D     : %16.16lf\n  MinSizeLength3D     : %16.16lf\n  MinSideLengthRatioUV: %16.16lf\n"),
             m_dMaximumSideLength3D,   
             m_dMinimumSideLength3D,    
             m_dMinimumSideLengthRatioUV) ;  
  smos_WriteBuffer(sBuff) ;

  // continuities
  if(m_pUContinuities)
    {
      smos_sprintf(sBuff, _T("\n  UContinuities Array[%ld] = {") ,m_pUContinuities->GetSize()) ;
      smos_WriteBuffer(sBuff) ;                                                             
      for(ii=0;ii<m_pUContinuities->GetSize();ii++) { 
          if(ii%10==0 && ii !=0) { 
              smos_WriteBuffer(_T("\n                             ")) ; 
          }
                                                      
          switch(m_pUContinuities->GetAt(ii))
          {
             case SM_CT_UNDEFINED     : smos_sprintf(sBuff, _T("%s"),_T(" SM_CT_UNDEFINED ") ); 
             case SM_CT_DISCONTINUOUS : smos_sprintf(sBuff, _T("%s"),_T(" SM_CT_DISCONTINUOUS ") ); 
             case SM_CT_C0            : smos_sprintf(sBuff, _T("%s"),_T(" SM_CT_C0 ") ); 
             case SM_CT_G1            : smos_sprintf(sBuff, _T("%s"),_T(" SM_CT_G1 ") ); 
             case SM_CT_G1R           : smos_sprintf(sBuff, _T("%s"),_T(" SM_CT_G1R ") );
             case SM_CT_G1_G2         : smos_sprintf(sBuff, _T("%s"),_T(" SM_CT_G1_G2 ") ); 
             case SM_CT_G1_G2_G3      : smos_sprintf(sBuff, _T("%s"),_T(" SM_CT_G1_G2_G3 ") ); 
             case SM_CT_C1            : smos_sprintf(sBuff, _T("%s"),_T(" SM_CT_C1 ") ); 
             case SM_CT_C1_G2         : smos_sprintf(sBuff, _T("%s"),_T(" SM_CT_C1_G2 ") );
             case SM_CT_C1_G2_G3      : smos_sprintf(sBuff, _T("%s"),_T(" SM_CT_C1_G2_G3 ") ); 
             case SM_CT_C1_C2         : smos_sprintf(sBuff, _T("%s"),_T(" SM_CT_C1_C2 ") ); 
             case SM_CT_C1_C2_G3      : smos_sprintf(sBuff, _T("%s"),_T(" SM_CT_C1_C2_G3 ") ); 
             case SM_CT_C1_C2_C3      : smos_sprintf(sBuff, _T("%s"),_T(" SM_CT_C1_C2_C3 ") ); 
             case SM_CT_CINFINITY     : smos_sprintf(sBuff, _T("%s"),_T(" SM_CT_CINFINITY ") );
             default : smos_sprintf(sBuff, _T("%s"),_T(" UnKnown Case ") );
          } 

          smos_WriteBuffer(sBuff) ;                                        
      }
      smos_WriteBuffer(_T("}"));
    }

  if(m_pVContinuities)
    {
      smos_sprintf(sBuff, _T("\n  VContinuities Array[%ld] = {") ,m_pVContinuities->GetSize()) ;
      smos_WriteBuffer(sBuff) ;
      for(ii=0;ii<m_pVContinuities->GetSize();ii++) { 
          if(ii%10==0 && ii !=0) { 
              smos_WriteBuffer(_T("\n                             ")) ; 
          }
                                                      
          switch(m_pVContinuities->GetAt(ii))
          {
             case SM_CT_UNDEFINED     : smos_sprintf(sBuff,_T("%s"), _T(" SM_CT_UNDEFINED ") ); 
             case SM_CT_DISCONTINUOUS : smos_sprintf(sBuff,_T("%s"), _T(" SM_CT_DISCONTINUOUS ") ); 
             case SM_CT_C0            : smos_sprintf(sBuff,_T("%s"), _T(" SM_CT_C0 ") ); 
             case SM_CT_G1            : smos_sprintf(sBuff,_T("%s"), _T(" SM_CT_G1 ") ); 
             case SM_CT_G1R           : smos_sprintf(sBuff,_T("%s"), _T(" SM_CT_G1R ") );
             case SM_CT_G1_G2         : smos_sprintf(sBuff,_T("%s"), _T(" SM_CT_G1_G2 ") ); 
             case SM_CT_G1_G2_G3      : smos_sprintf(sBuff,_T("%s"), _T(" SM_CT_G1_G2_G3 ") ); 
             case SM_CT_C1            : smos_sprintf(sBuff,_T("%s"), _T(" SM_CT_C1 ") ); 
             case SM_CT_C1_G2         : smos_sprintf(sBuff,_T("%s"), _T(" SM_CT_C1_G2 ") );
             case SM_CT_C1_G2_G3      : smos_sprintf(sBuff,_T("%s"), _T(" SM_CT_C1_G2_G3 ") ); 
             case SM_CT_C1_C2         : smos_sprintf(sBuff,_T("%s"), _T(" SM_CT_C1_C2 ") ); 
             case SM_CT_C1_C2_G3      : smos_sprintf(sBuff,_T("%s"), _T(" SM_CT_C1_C2_G3 ") ); 
             case SM_CT_C1_C2_C3      : smos_sprintf(sBuff,_T("%s"), _T(" SM_CT_C1_C2_C3 ") ); 
             case SM_CT_CINFINITY     : smos_sprintf(sBuff,_T("%s"), _T(" SM_CT_CINFINITY ") );
             default : smos_sprintf(sBuff,_T("%s"), _T(" UnKnown Case ") );
          } 
          smos_WriteBuffer(sBuff) ;
                                                    
      }
      smos_WriteBuffer(_T("}"));
    }

  // tessellation algorithm 
  smos_sprintf(sBuff, _T("\n  TessellationAlgorithm       : %s"),
               (m_eTessellationAlgorithm == SM_TA_FASTER_TESSELLATION) ? _T("SM_TA_FASTER_TESSELLATION")
             : (m_eTessellationAlgorithm == SM_TA_FEWER_POLYGONS) ? _T("SM_TA_FEWER_POLYGONS")
             : _T("UnKnown")) ;  
  smos_WriteBuffer(sBuff) ;

  // tessellation optimization parameters
  smos_sprintf(sBuff, _T("\n  StartFastSubdivisionAngleDeg: %16.16lf, SwitchToFastSubdivision: %s"),
             m_dStartFastSubdivisionAngleDeg,
             m_bSwitchToFastSubdivision ? _T("TRUE") : _T("FALSE") );   
  smos_WriteBuffer(sBuff) ;

  // tessellation optimization parameters
  smos_sprintf(sBuff, _T("\n  TrimSubdivisionFactor       : %16.16lf - (split LeafNodes until UVTrimCurve-Segment Ctrl-Polygon BBox widths < UVDomainSize/TrimSubdivisionFactor)"), m_dTrimSubdivisionFactor );   
  smos_WriteBuffer(sBuff) ; 

  // Memory sized           
  smos_sprintf(sBuff, _T("\n  MemorySizes                 : m_lBezierSize = %ld, m_lTotalBezSize = %ld"), m_lBezierSize, m_lTotalBezSize) ;
  smos_WriteBuffer(sBuff) ;
                            
  smos_sprintf(sBuff, _T("\n  Tmp Val IsoCurveTolerance   : %16.16lf (Last ApproxTol3d val used to produce an IsoParamCurve)"), m_dIsoCurveTolerance) ;  
  smos_WriteBuffer(sBuff) ;

  // Bounding planes        
  smos_sprintf(sBuff, _T("\n  BoundingPlanes              : %s (%s\n                                            %s)"),
             m_sBoundingPlanes.ArePlanesComputed() ? _T("Defined") : _T("NotDefined") ,
             _T("array of boundary planes defined when 3D region boundary curve is"),
             _T("planar and the surface lies completely to one side of the plane.") );   
  smos_WriteBuffer(sBuff) ;

  // something to dump the actual boundary planes needed

  // m_bHaveTSurfaceCache   
  smos_sprintf(sBuff, _T("\n  HaveTSurfaceCache           : %s (%s\n                          %s\n                          %s)"),
             m_bHaveTSurfaceCache ?  _T("TRUE") : _T("FALSE") ,
             _T("Initial: FALSE, Set to True by SmTrimSrfCache::BuildTree when"),
             _T("                      Subdivision Tree is augmented with object list"),
             _T("                      pointers to face boundary UVTrimCurves and Vertices.") );   
  smos_WriteBuffer(sBuff) ;

  // m_bFaceWasModified, m_bProcessBoundaryCurves, m_bPointTestEnabled, m_bFaceContainmentDone
  smos_sprintf(sBuff, _T("\n  FaceWasModified      : %s\n  ProcessBoundaryCurves: %s\n  PointTestEnabled     : %s\n  FaceContainmentDone  : %s"),
             m_bFaceWasModified       ?  _T("TRUE") : _T("FALSE") ,
             m_bProcessBoundaryCurves ?  _T("TRUE") : _T("FALSE") ,
             m_bPointTestEnabled      ?  _T("TRUE") : _T("FALSE") ,
             m_bFaceContainmentDone   ?  _T("TRUE") : _T("FALSE") );   
  smos_WriteBuffer(sBuff) ;

  // the tree
  if(m_pTree) { m_pTree->Dump() ; }
  
  smos_WriteBuffer(_T("\nEnd SmSurfaceCache Dump()"));

} // end SmSurfaceCache::Dump

/********************************************************************
PURPOSE: Render the Cache decomposition of a surface with or without
            optional bounding boxes by drawing each SurfaceCache TreeNode
            decomposed bezier patch outline as a simplified set of 4 corners
            connected by line segments

NOTES:
   1. Internal discontinuities are drawn double wide in cyan.
   2. When bDrawTree == TRUE draw all tree Node bounding boxes.
   3. 
*********************************************************************/
SmDisplayList * SmSurfaceCache::Draw
  (SmBoolean bDrawTree,                    // in : TRUE = Draw every treeNode's bounding box
                                           //      FALSE= Don't
   SmBoolean bDrawBoundingPlanes,          // in : TRUE = Draw Bounding planes for Surface region 
                                           //             when they exist (no boundaries for nonPlanar 3dBoundary curves)
                                           //             or when surface is not bounded by boundary.
                                           //      FALSE= don't
   SmBoolean bOnlyDrawNodesWithVertices,   // in : TRUE = When drawing treenode's bounding boxes, only draw those containing vertices
   SmBoolean bOnlyDrawNodesWithEdges,      // in : TRUE = when drawing treenode's bounding boxes, only draw those containing edges
   SmBoolean bOnlyDrawLeafNodes)           // in : TRUE = skip drawing parent nodes

 const
{
  SmDisplayList *pRtn = NULL ;

#ifdef SM_GFX_CODE
  // start new DisplayList (unless one is already open)
  SmVector3d sColor = smgfx_GetOutputColor() ;
  smgfx_Open(smgfx_GetRuleColor(m_cpSurface));

  // when asked - draw all tree node bounding boxes 
  if (bDrawTree) 
    {
      if (!m_pTree) return(NULL);

      // visit all nodes in the tree
      SmTArray<SmTreeNode*> sStack;
      sStack.Add(m_pTree->GetTopNode());
      while (sStack.GetSize() > 0) 
        {
          SmTreeNode    *pNode = (SmTreeNode*)sStack.GetLast();
          SmBezierAux2d *pAux  = (SmBezierAux2d*)pNode->m_pData;
          SmBoolean      bDraw = TRUE ;

          // when asked skip nodes without vertices and edges
          if(   bOnlyDrawNodesWithVertices == TRUE
             || bOnlyDrawNodesWithEdges    == TRUE)
            {
              SmTArray<SmVertexList*> sVertexListArr ;
              SmTArray<SmEdgeuseList*>  sEdgeuseListArr ;
              SmTArray<SmPolyEdgeList*> sPolyEdgeListArr;
              pAux->m_vVertexList.GetAllNodes(sVertexListArr);
              pAux->m_sEdgeuseList.GetAllNodes(sEdgeuseListArr);
              pAux->m_sPolyEdgeList.GetAllNodes(sPolyEdgeListArr);

              SmBoolean bNoVertices = sVertexListArr.GetSize() == 0 ;
              SmBoolean bNoEdges = ((sEdgeuseListArr.GetSize() == 0) && (sPolyEdgeListArr.GetSize() == 0));



              if(   bOnlyDrawNodesWithVertices == TRUE
                 && bOnlyDrawNodesWithEdges    == TRUE) { if(bNoVertices && bNoEdges) bDraw = FALSE ; }
              else if(bOnlyDrawNodesWithVertices)       { if(bNoVertices)             bDraw = FALSE ; }
              else if(bOnlyDrawNodesWithEdges)          { if(bNoEdges)                bDraw = FALSE ; }
            }

          // when asked skip parent nodes
          if(   bOnlyDrawLeafNodes 
             && (   pNode->m_pChild1 != NULL
                 || pNode->m_pChild2 != NULL))
            { bDraw = FALSE ; }

          // draw the node's bounding box
          if(bDraw) 
            { 
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
              if(bDebugMe)
                {
                  pNode->m_sBBox.Dump() ;

                  if(   (   pNode->m_eAuxDataType == SM_AD_BEZIER_SURFACE_HEAD
                         || pNode->m_eAuxDataType == SM_AD_AUX_DATA)
                     && pNode->m_pData)
                     {
                       SmBezierAux2d *pBezAux = (SmBezierAux2d*)pNode->m_pData;
                       smos_WriteBuffer(_T("\n  ")) ; pBezAux->GetUVDomain().Dump() ;
                     }          
                }
#endif // SM_DEBUG_CODE
              pNode->m_sBBox.Draw(GetContext());
              smgfx_ChangeColor();
            }
             
          sStack.RemoveLast();
          if (pNode->m_pChild1) 
            {
              sStack.Add(pNode->m_pChild1);
              sStack.Add(pNode->m_pChild2);
            }
        } // end while nodes left to visit

      // restore the original draw color
      smgfx_SetColor(sColor) ;

    } // end draw tree node bounding boxes check

  // when asked - draw the surface bounding planes
  if(bDrawBoundingPlanes)
    {
      m_sBoundingPlanes.Draw() ;
    } // end draw boundingPlanes check

  // get all tree nodes
  if(m_pTree)
    {
      SmTArray<void*> sNodes;
      m_pTree->m_sNodeMgr.GetActiveElements(sNodes);
  
      // for every tree node - draw the bezierSurface outline
      for (ULONG i=0; i<sNodes.GetSize(); i++) 
        {
          SmTreeNode *pNode = (SmTreeNode*)sNodes[i];

          // skip all but Bezier Surface leaf nodes
          if (pNode->m_eAuxDataType != SM_AD_BEZIER_SURFACE) continue;
          if (pNode->m_pChild1 != NULL) continue;

          // leaf node surface bezier patch locals
          SmBezierPatch * pBezElem = (SmBezierPatch*)pNode->m_pData;
          const SmSurface *pBSS =   pBezElem->mBA_pSurface 
                                  ? pBezElem->mBA_pSurface 
                                  : this->m_cpSurface ;

          SmExtent2d sUVDomain = pBezElem->GetUVDomain();

          // watch out for NULL surfaces
          if(pBSS)
            {
              // Add in extra thick edges for
              // boundaries which are not G1 or better
              for (ULONG j=0; j<=1; j++) 
                {
                  SmPoint3d sP1, sP2;
                  if (pBezElem->m_eUCurveConts[j] < SM_CT_G1) 
                    {
                      double     dLineWidth = smgfx_SetLineWidth(3.0*smgfx_GetLineWidth());
                      pBSS->EvaluatePoint(sUVDomain.Evaluate(j,0),sP1);
                      pBSS->EvaluatePoint(sUVDomain.Evaluate(j,1),sP2);
                      smgfx_DrawLine(sP1.x,sP1.y,sP1.z,sP2.x,sP2.y,sP2.z);
                      smgfx_SetLineWidth(dLineWidth) ;
                    }
                  if (pBezElem->m_eVCurveConts[j] < SM_CT_G1) 
                    {
                      double     dLineWidth = smgfx_SetLineWidth(3.0*smgfx_GetLineWidth());
                      pBSS->EvaluatePoint(sUVDomain.Evaluate(0,j),sP1);
                      pBSS->EvaluatePoint(sUVDomain.Evaluate(1,j),sP2);
                      smgfx_DrawLine(sP1.x,sP1.y,sP1.z,sP2.x,sP2.y,sP2.z);
                      smgfx_SetLineWidth(dLineWidth);
                    }
                }

              // draw the patch domain projected through the surface
              SmExtent2d sShrunkUVDomain = sUVDomain ;
              sShrunkUVDomain.ExpandRelative(.90) ;
              pBSS->DrawUVDomain(sShrunkUVDomain) ;
            }
        } // end iter every tree node
    } // end m_pTree existence check

  //end DisplayList
  pRtn = smgfx_Close() ;

#else
  SM_REF5(bDrawTree, bDrawBoundingPlanes, bOnlyDrawNodesWithVertices, bOnlyDrawNodesWithEdges, bOnlyDrawLeafNodes);
#endif // SM_GFX_CODE
  return(pRtn) ;

} // end SmSurfaceCache::Draw

/*******************************************************************//**
PURPOSE: Draw SmTrimSrfCache subdivision tree leafnode boundaries
  and midPoints in 2d.

NOTES:  leafnode midPoints are assigned the following colors
  to show in/out/on classification.

  MidPoint Colors
  -------------------
  red   = ON_BOUNDARY
  green = INSIDE
  blue  = OUTSIDE
  black = UNKNOWN

***********************************************************************/
SmDisplayList * SmSurfaceCache::DrawSubdivision2D
  (SmBoolean       bDrawGeometry,              // in : TRUE= Also draw Face->UVTrimCurves and UVVertexPts on plane
   SmBoolean       bOnlyDrawNodesWithVertices, // in : TRUE= Draw only Subdivision Nodes that contain Face vertices
   SmBoolean       bOnlyDrawNodesWithEdges,    // in : TRUE= Draw only Subdivision Nodes that contain Face edges
   SmPlane       * pOptOutPlane,               // in : Draw on this plane, NULL=draw on z=0 plane, default:[NULL]
   SmGfxArraySet * pOptGfxSet)                 // i/o: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.
                                               //      NULL to ignore. default:[NULL]
 const
{
  SmDisplayList *pRtn = NULL ;

#ifdef SM_GFX_CODE

  // open display list
  smgfx_Open(smgfx_GetRuleColor(this), NULL, NULL, FALSE, pOptGfxSet);

  // when asked - Draw Face->Edge->UVTrimCurves and Face->Vertex->UVPoints on z=0 plane
  if (bDrawGeometry) 
    {
      // Now implant edgeuses and vertexuses into the cache tree
      SmFaceuse *pFU = GetFace()->GetUpwardFaceuse();
      SmTArray<SmLoopuse*> sLoopuses;
      pFU->GetLoopuses(sLoopuses);
      SmTArray<SmVertexuse*> sVUses;
      SmTArray<SmEdgeuse*> sEUses;

      // for every loopuse
      for (ULONG i=0; i<sLoopuses.GetSize(); i++) 
        {
          SmLoopuse *pLU = (SmLoopuse*)sLoopuses[i];
          
          // Draw every Vertexuse UVPoint
          pLU->GetVertexuses(sVUses);
          for (ULONG k=0; k<sVUses.GetSize(); k++) 
            {
              SmVertexuse *pVertexuse = sVUses[k];
              SmPoint2d sUV;
              pVertexuse->ComputeUVPoint(sUV);
              SmPoint3d sPnt(sUV);
              if(pOptOutPlane)
                { pOptOutPlane->EvaluatePoint(sUV, sPnt) ; }
              sPnt.Draw(NULL, NULL, pOptGfxSet);
            }

          // Draw every Edgeuse->UVTrimCurve
          pLU->GetEdgeuses(sEUses);
          for (ULONG j=0; j<sEUses.GetSize(); j++) 
            {
              SmEdgeuse * pEU    = sEUses[j];
              SmCurve   * pCurve = pEU->GetUVTrimCurve();
              pCurve->Draw(NULL, FALSE, pOptOutPlane, pOptGfxSet) ;    
            }
        } // end iter every loopuse
    } // end bDrawGeometry == TRUE check

  SmTArray<void*> sNodes;
  m_pTree->m_sNodeMgr.GetActiveElements(sNodes);

  // draw every SmTrimSrfCache leafnode boundary and midPoint
  // midPoint color red   = ON_BOUNDARY
  //                green = INSIDE
  //                blue  = OUTSIDE
  //                black = UNKNOWN
  for (ULONG i=0; i<sNodes.GetSize(); i++) 
    {
      SmTreeNode *pNode = (SmTreeNode*)sNodes[i];

      // skip nonSurface subdivision nodes
      if(   pNode->m_eAuxDataType != SM_AD_BEZIER_SURFACE
         && pNode->m_eAuxDataType != SM_AD_AUX_DATA
         && pNode->m_eAuxDataType != SM_AD_BEZIER_SURFACE_HEAD) 
        { continue; }

      // skip nonLeaf nodes
      if (pNode->m_pChild1 != NULL) 
        { continue; }

      // All Surface node m_pData types are derived from base SmBezierAux2d
      SmBezierAux2d *pAux = (SmBezierAux2d*)pNode->m_pData;

      // when asked - see if node is to be skipped
      if(   bOnlyDrawNodesWithVertices == TRUE
         || bOnlyDrawNodesWithEdges    == TRUE)
        {
          SmTArray<SmVertexList*> sVertexListArr ;
          SmTArray<SmEdgeuseList*>  sEdgeuseListArr ;
          SmTArray<SmPolyEdgeList*> sPolyEdgeListArr;
          pAux->m_vVertexList.GetAllNodes(sVertexListArr);
          pAux->m_sEdgeuseList.GetAllNodes(sEdgeuseListArr);
          pAux->m_sPolyEdgeList.GetAllNodes(sPolyEdgeListArr);

          // remember if node has vertices and/or edges
          SmBoolean bNoVertices = sVertexListArr.GetSize() == 0 ;
          SmBoolean bNoEdges    = ((sEdgeuseListArr.GetSize() == 0) && (sEdgeuseListArr.GetSize())) ;

          // skip nodes without vertices and/or edges
          if(   (   bOnlyDrawNodesWithVertices == TRUE && bNoVertices 
                 && bOnlyDrawNodesWithEdges    == TRUE && bNoEdges)
             || (   bOnlyDrawNodesWithVertices         && bNoVertices)
             || (   bOnlyDrawNodesWithEdges            && bNoEdges))                
           { continue ; } 
        
        } // end need to skip node check

      // Draw black leafnode domain boundary
      smgfx_SetColor(0,0,0, pOptGfxSet);
      pAux->m_sUVDomain.Draw(GetContext(), pOptOutPlane, pOptGfxSet);

      // draw leafnode domain boundary midPoint
      //       red   = ON_BOUNDARY
      //       green = INSIDE
      //       blue  = OUTSIDE
      //       black = UNKNOWN
      if     (pAux->m_eNodeClass == SM_NC_ON_BOUNDARY) { smgfx_SetColor(1,0,0, pOptGfxSet); }
      else if(pAux->m_eNodeClass == SM_NC_INSIDE)      { smgfx_SetColor(0,1,0, pOptGfxSet); }
      else if(pAux->m_eNodeClass == SM_NC_OUTSIDE)     { smgfx_SetColor(0,0,1, pOptGfxSet); }
      else                                             { smgfx_SetColor(0,0,0, pOptGfxSet); }
      SmPoint3d sMid(pAux->m_sUVDomain.Evaluate(0.5,0.5));
      if(pOptOutPlane)
        { 
          SmPoint2d sMidUV(sMid.x, sMid.y) ;
          pOptOutPlane->EvaluatePoint(sMidUV,sMid) ;
        }

      sMid.Draw(NULL, NULL, pOptGfxSet);
      smgfx_SetColor(0,0,0, pOptGfxSet);

    } // end drawing every SmTrimSrfCache node boundary and midPoint

  pRtn = smgfx_Close(pOptGfxSet) ;

#else
  SM_REF5(bDrawGeometry, bOnlyDrawNodesWithVertices, bOnlyDrawNodesWithEdges, pOptOutPlane, pOptGfxSet);
#endif // end SM_GFX_CODE

  return(pRtn) ;

} // end SmSurfaceCache::DrawSubdivision2D

/*******************************************************************//**
PURPOSE: Draw SmTrimSrfCache subdivision tree leafnode bounding boxes.

NOTES:  leafnode bounding boxes are assigned the following colors
  to show in/out/on classification.

  MidPoint Colors
  -------------------
  red   = ON_BOUNDARY
  green = INSIDE
  blue  = OUTSIDE
  black = UNKNOWN

***********************************************************************/
SmDisplayList * SmSurfaceCache::DrawSubdivision3D
  (SmBoolean       bOnlyDrawNodesWithVertices, // in : TRUE=Only Draw Nodes containing SmVertices, FALSE=draw all nodes, default:[FALSE]
   SmBoolean       bOnlyDrawNodesWithEdges,    // in : TRUE=Only Draw Nodes containing SmEdges, FALSE=draw all nodes, default:[FALSE]
   SmGfxArraySet * pOptGfxSet)                 // i/o: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.
                                               //      NULL to ignore. default:[NULL]
 const
{
  SmDisplayList *pRtn = NULL ;

#ifdef SM_GFX_CODE

  // open display list
  smgfx_Open(smgfx_GetRuleColor(this), NULL, NULL, FALSE, pOptGfxSet);

  // draw every SmTrimSrfCache leafnode boundary and midPoint
  // midPoint color red   = ON_BOUNDARY
  //                green = INSIDE
  //                blue  = OUTSIDE
  //                black = UNKNOWN
  SmTArray<void*> sNodes;
  m_pTree->m_sNodeMgr.GetActiveElements(sNodes);
  for (ULONG i=0; i<sNodes.GetSize(); i++) 
    {
      SmTreeNode    * pNode = (SmTreeNode*)sNodes[i];
      SmBezierAux2d * pAux  = (SmBezierAux2d*)pNode->m_pData;
          
      // skip nonSurface subdivision nodes
      if(   pNode->m_eAuxDataType != SM_AD_BEZIER_SURFACE
         && pNode->m_eAuxDataType != SM_AD_AUX_DATA
         && pNode->m_eAuxDataType != SM_AD_BEZIER_SURFACE_HEAD) 
        { continue; }

      // skip nonLeaf nodes
      if (pNode->m_pChild1 != NULL) 
        { continue; }

      // when asked skip nodes without vertices and edges
      if(   bOnlyDrawNodesWithVertices == TRUE
         || bOnlyDrawNodesWithEdges    == TRUE)
        {
          SmTArray<SmVertexList*> sVertexListArr ;
          SmTArray<SmEdgeuseList*> sEdgeuseListArr;
          SmTArray<SmPolyEdgeList*> sPolyEdgeListArr;
          pAux->m_vVertexList.GetAllNodes(sVertexListArr);
          pAux->m_sEdgeuseList.GetAllNodes(sEdgeuseListArr);
          pAux->m_sPolyEdgeList.GetAllNodes(sPolyEdgeListArr);

          SmBoolean bNoVertices = sVertexListArr.GetSize() == 0 ;
          SmBoolean bNoEdges = ((sEdgeuseListArr.GetSize() == 0) && (sPolyEdgeListArr.GetSize()));

          // when asked - skip Nodes without Vertices and/or Edges
          if(   (   bOnlyDrawNodesWithVertices == TRUE
                 && bOnlyDrawNodesWithEdges    == TRUE && (bNoVertices && bNoEdges))
             || (   bOnlyDrawNodesWithVertices == TRUE && (bNoVertices))
             || (   bOnlyDrawNodesWithEdges    == TRUE && (bNoEdges)))
            { continue ; }
        }

      // draw bounding box and midPoint node
      // pNode->Draw(TRUE, NULL, pOptGfxSet) ;

      // draw UVDomain on Surface 
      pAux->m_sUVDomain.Draw(GetContext(), GetSurface(), pOptGfxSet);

      //      SmPoint3d sMidPoint ;
      //      SmPoint2d sPointUV ;
      //      // for leafNodes - m_pData is of type SmBezierPatch
      //      SmBezierPatch *pBezElem = (SmBezierPatch*)pNode->m_pData;
      //      SmBezierAux2d     *pAux     = pBezElem;
      //      
      //      
      //      // draw leafnode domain boundary midPoint
      //      // color red   = ON_BOUNDARY
      //      //       green = INSIDE
      //      //       blue  = OUTSIDE
      //      //       black = UNKNOWN
      //      if     (pAux->m_eNodeClass == SM_NC_ON_BOUNDARY) { smgfx_SetColor(1,0,0); }
      //      else if(pAux->m_eNodeClass == SM_NC_INSIDE)      { smgfx_SetColor(0,1,0); }
      //      else if(pAux->m_eNodeClass == SM_NC_OUTSIDE)     { smgfx_SetColor(0,0,1); }
      //      else                                             { smgfx_SetColor(0,0,0); }
      //      
      //      pNode->m_sBBox.Draw() ;
      //      
      //      // add center point - evaluate through uv_domain because some BBoxes are
      //      //   just equal to their parent's bbox
      //      // GWC TODO: Once the child BBox problem is fixed - just
      //      //           evaluate the BBox center - its a lot cheaper.
      //      sPointUV  = pAux->m_sUVDomain.Evaluate(.5,.5) ;
      //      
      //      gw_SURFACE *pSur = pNode->GetFirstBezierPatch() ;
      //      SmBSplineSurface sBSS(pSur, TRUE, pContext, FALSE) ; // FALSE = don't run AssertValidAndHeal on New BSplineSurface
      //      sBSS.EvaluatePoint(sPointUV, sMidPoint) ;
      //      
      //      sMidPoint.Draw() ;

    } // end drawing every SmTrimSrfCache node boundary and midPoint

  pRtn = smgfx_Close(pOptGfxSet) ;

#else
  SM_REF3(bOnlyDrawNodesWithVertices, bOnlyDrawNodesWithEdges, pOptGfxSet);
#endif // end SM_GFX_CODE
  return(pRtn) ;

} // end SmSurfaceCache::DrawSubdivision3D


// obsolete
//  /*******************************************************************//**
//  PURPOSE:
//  
//  NOTES:
//  ***********************************************************************/
//  SmDisplayList * SmSurfaceCache::DrawSubdivision2D
//   (SmPlane       * pOptOutPlane,  // in : Draw on this plane, NULL=draw on z=0 plane, default:[NULL]
//    SmGfxArraySet * pOptGfxSet)    // i/o: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.
//                                   //      NULL to ignore. default:[NULL]
//   const
//  {
//    SmDisplayList *pRtn = NULL ;
//  
//  #ifdef SM_GFX_CODE
//    // start new displayList (unless one is already open)
//    SmVector3d sColor = smgfx_GetOutputColor(pOptGfxSet) ;
//    smgfx_Open(smgfx_GetRuleColor(this), NULL, NULL, FALSE, pOptGfxSet);
//  
//    SM_PTR_ARRAY(sNodes, SmTreeNode, 100) ; // SmTArray<SmTreeNode*>
//    SE(FindUVNodes(m_sUVDomain,sNodes));
//    
//    // for every surface subdivision node
//    for (ULONG i=0; i<sNodes.GetSize(); i++) 
//      {
//        SmTreeNode *pNode = sNodes[i];
//  
//        // skip non surface subdivision nodes
//        if(   pNode->m_eAuxDataType != SM_AD_BEZIER_SURFACE
//           && pNode->m_eAuxDataType != SM_AD_AUX_DATA
//           && pNode->m_eAuxDataType != SM_AD_BEZIER_SURFACE_HEAD) 
//          { continue; }
//  
//        // skip parent nodes
//        if(pNode->m_pChild1 != NULL) 
//          { continue; }
//  
//        // arrive here when node is a subdivision leaf node
//        // it always has an m_pData which is at lease an SmBezierAux2d
//        //  (sometimes - but not always - m_pData will be SmBezierPatch)
//        SmBezierAux2d *pAux = (SmBezierAux2d *)pNode->m_pData;
//        SmPoint3d      sMid(pAux->m_sUVDomain.Evaluate(0.5,0.5));
//            
//        // Draw a black UVBox
//        smgfx_SetColor(0,0,0, pOptGfxSet);
//        pAux->m_sUVDomain.Draw(GetContext(), pOptOutPlane, pOptGfxSet);
//  
//        // Draw classification MidPoints a different color
//        if      (pAux->m_eNodeClass == SM_NC_ON_BOUNDARY) { smgfx_SetColor(1,0,0, pOptGfxSet); }
//        else if (pAux->m_eNodeClass == SM_NC_INSIDE)      { smgfx_SetColor(0,1,0, pOptGfxSet); }
//        else if (pAux->m_eNodeClass == SM_NC_OUTSIDE)     { smgfx_SetColor(0,0,1, pOptGfxSet); }
//  
//        if(pOptOutPlane)
//          {
//            SmPoint2d sMidUV(sMid.x, sMid.y) ;
//            pOptOutPlane->EvaluatePoint(sMidUV,sMid) ;
//          }
//  
//        sMid.Draw(NULL, NULL, pOptGfxSet);
//        smgfx_SetColor(0,0,0, pOptGfxSet);
//  
//      } // end iter every tree node looking for childrent to draw
//  
//    // end DisplayList
//    smgfx_OutputColor(sColor, pOptGfxSet) ;
//    pRtn = smgfx_Close(pOptGfxSet) ;
//  
//  #endif // SM_GFX_CODE
//    return(pRtn) ;
//  
//  } // end SmSurfaceCache::DrawSubdivision2D
//  
//  /*******************************************************************//**
//  PURPOSE:
//  
//  NOTES:
//  ***********************************************************************/
//  SmDisplayList * SmSurfaceCache::DrawSubdivision3D
//   (SmGfxArraySet * pOptGfxSet)    // i/o: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.
//                                   //      NULL to ignore. default:[NULL]
//   const
//  {
//    SmDisplayList *pRtn = NULL ;
//  
//  #ifdef SM_GFX_CODE
//    // locals
//    const SmSurface * pSurface = m_cpSurface ;
//    SmPoint3d         sMid;
//    SM_PTR_ARRAY     (sNodes, SmTreeNode, 100) ; // SmTArray<SmTreeNode*>
//    SE(FindUVNodes(m_sUVDomain,sNodes));
//    
//    // start new displayList (unless one is already open)
//    SmVector3d sColor = smgfx_GetOutputColor(pOptGfxSet) ;
//    smgfx_Open(smgfx_GetRuleColor(this), NULL, NULL, FALSE, pOptGfxSet);
//  
//    // for every surface subdivision node
//    for (ULONG i=0; i<sNodes.GetSize(); i++) 
//      {
//        SmTreeNode *pNode = sNodes[i];
//  
//        // skip non surface subdivision nodes
//        if(   pNode->m_eAuxDataType != SM_AD_BEZIER_SURFACE
//           && pNode->m_eAuxDataType != SM_AD_AUX_DATA
//           && pNode->m_eAuxDataType != SM_AD_BEZIER_SURFACE_HEAD) 
//          { continue; }
//  
//        // skip parent nodes
//        if(pNode->m_pChild1 != NULL) 
//          { continue; }
//  
//        // arrive here when node is a subdivision leaf node
//        // it always has an m_pData which is at lease an SmBezierAux2d
//        //  (sometimes - but not always - m_pData will be SmBezierPatch)
//        SmBezierAux2d *pAux = (SmBezierAux2d *)pNode->m_pData;
//  
//        // draw a black UV BBox projected through the surface
//        smgfx_SetColor(0,0,0, pOptGfxSet);
//        pSurface->DrawUVBox(pAux->m_sUVDomain, pOptGfxSet) ;
//  
//        // Draw different classifications different midPoint colors
//        pSurface->EvaluatePoint(pAux->m_sUVDomain.Evaluate(0.5,0.5), sMid);
//        if (pAux->m_eNodeClass == SM_NC_ON_BOUNDARY) 
//          {
//            smgfx_SetColor(1,0,0, pOptGfxSet);
//            sMid.Draw(NULL, NULL, pOptGfxSet);
//            smgfx_SetColor(0,0,0, pOptGfxSet);
//          }
//        else if (pAux->m_eNodeClass == SM_NC_INSIDE) 
//          {
//            smgfx_SetColor(0,1,0);
//            sMid.Draw(NULL, NULL, pOptGfxSet);
//            smgfx_SetColor(0,0,0);
//          }
//        else if (pAux->m_eNodeClass == SM_NC_OUTSIDE) 
//          {
//            smgfx_SetColor(0,0,1);
//            sMid.Draw(NULL, NULL, pOptGfxSet);
//            smgfx_SetColor(0,0,0);
//          }
//      } // end iter every tree node looking for childrent to draw
//  
//    // end DisplayList
//    smgfx_OutputColor(sColor, pOptGfxSet) ;
//    pRtn = smgfx_Close(pOptGfxSet) ;
//  
//  #endif // SM_GFX_CODE
//    return(pRtn) ;
//  
//  } // end SmSurfaceCache::DrawSubdivision3D

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
void SmVertexList::Dump() const
{
  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE];
  
  smos_sprintf(sBuff,        _T("SmVertexList 0x%p"), this);
  smos_sprintf(sBuffForFile,_T("%s"), _T("SmVertexList"));
  smos_WriteBuffer(sBuff, sBuffForFile);
  smos_WriteBuffer(_T("\n  m_pVU      : ")) ; if(m_pVU) m_pVU->Dump() ;
  smos_WriteBuffer(_T("\n  m_vUVPoint : ")) ; m_vUVPoint.Dump() ;
  smos_WriteBuffer(_T("\n  m_vUVTol   : ")) ; m_vUVTol.Dump() ; 

} // end SmVertexList::Dump

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
void SmEdgeuseList::Dump() const
{
  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE];
  
  smos_sprintf(sBuff,        _T("SmEdgeuseList 0x%p"), this);
  smos_sprintf(sBuffForFile, _T("%s"),_T("SmEdgeuseList "));
  smos_WriteBuffer(sBuff, sBuffForFile);
  smos_WriteBuffer(_T("\n  m_pEU      : ")) ;   
  if(m_pEU) m_pEU->Dump() ;

  // not used: smos_WriteBuffer(_T("\n  m_pCurveNode : ")) ; if(m_pCurveNode) m_pCurveNode->Dump() ;

} // end SmEdgeuseList::Dump
/*******************************************************************/ /**
 PURPOSE:

 NOTES:
 ***********************************************************************/
void SmPolyEdgeList::Dump() const
{
    TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE];

    smos_sprintf(sBuff, _T("SmPolyEdgeList 0x%p"), this);
    smos_sprintf(sBuffForFile,_T("%s"), _T("SmPolyEdgeList "));
    smos_WriteBuffer(sBuff, sBuffForFile);
    smos_WriteBuffer(_T("\n  m_pPolyEdge      : "));
    if (m_pPolyEdge) m_pPolyEdge->Dump();

    // not used: smos_WriteBuffer(_T("\n  m_pCurveNode : ")) ; if(m_pCurveNode) m_pCurveNode->Dump() ;

} // end SmPolyEdgeList::Dump
