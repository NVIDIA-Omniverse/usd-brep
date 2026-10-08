// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmRayTracer.cpp
* PURPOSE: Source file for SmRayTracer methods.
**********************************************************************/

#include "StdAfx.h"


// This is required to recognize SmPointClassification::GetObject after gl is included.
#ifdef GetObject
  #undef GetObject
#endif


#include <SmRayTracer.h>


 #include <SmBSplineCurve.h>
 #include <SmBSplineSurface.h>
 #include <SmLine.h>
 #include <SmNurbsSrf.h>
 #include <SmFace.h>
 #include <SmBrep.h>

#include <SmPoly.h>


#include <SmGraphicsExtern.h>
#include <SmSolutionArray.h>
#include <SmTree.h>

#include <SmGeomUtility.h>
#include <SmGraphicsOutput.h>

#include <SmCacheMgr.h>
#include <SmBrepCache.h>
#include <SmSurfaceCache.h>

#ifdef SM_DEBUG_CODE
 #include <SmPlane.h>
#endif

#include <time.h>

/*******************************************************************//**
PURPOSE: Find line/Plane intersection

RETURNS --- SM_SUCCESS == Line intersects Plane
            SM_ERR     == Line does not intersect plane(parallel)
***********************************************************************/
static SmStatus sm_LinePlaneIntersect
(const SmPoint3d    & crLinePnt,          // in : pt  of Line = pt + T * Vec
   const SmVector3d & crLineVec,          // in : vec of Line = pt + T * Vec
   const SmPoint3d  & crPlanePnt,         // in : Pt    of Plane = Pt + U * XVec + V * YVec
   const SmVector3d & crPlaneNorm,        // NotUsed: in : Plane Normal = XVec * YVec
   const SmVector3d & crPlaneU,           // in : XVec  of Plane = Pt + U * XVec + V * YVec  
   const SmVector3d & crPlaneV,           // in : YVec  of Plane = Pt + U * XVec + V * YVec  
   SmPoint3d        & rIntersectPoint,    // out: 3d intersection point
   SmPoint2d        & rUVIntersectPoint,  // out: Plane UV intersection point
   double           & rdLineParam)        // out: Line param intersection value
{
  SM_REF1(crPlaneNorm) ; 
  SmStatus eStat = smgu_LinePlaneIntersect(
    crLinePnt,
    crLineVec,
    crPlanePnt,
    crPlaneU,
    crPlaneV,
    rdLineParam,
    rUVIntersectPoint.x,
    rUVIntersectPoint.y );

  if ( eStat == SM_SUCCESS )
    { rIntersectPoint = crLinePnt + rdLineParam * crLineVec; }

  return eStat;

} // end sm_LinePlaneIntersect


/*******************************************************************//**
PURPOSE: Constructor for the SmRayRenderer class. The SmRayRenderer
   will now own the RayTracer and destroy it when done.

NOTES: 
***********************************************************************/
SmRayRenderer::SmRayRenderer
(SmRayTracer * pRayTracer,
   const SmAxis2Placement & crGeometryTransform,
   const SmExtent3d & crViewVolume,
   ULONG lNumXPixels,
   ULONG lNumYPixels)
 : m_pRayTracer(pRayTracer),
   m_vGeometryTransform(crGeometryTransform), 
   m_vViewVolume(crViewVolume),
   m_lNumXPixels(lNumXPixels),
   m_lNumYPixels(lNumYPixels),
   m_lSampleRate(1),
   m_bDoShadows(FALSE),
   m_lPixelsShaded(0)
{
    m_vGeometryTransform.Invert(m_vInvTransform);
} // end SmRayRenderer::SmRayRenderer constructor


/*******************************************************************//**
PURPOSE: Destructor for the SmRayRenderer class.

NOTES: 
***********************************************************************/
SmRayRenderer::~SmRayRenderer()
{
    ULONG ii;
    for (ii = 0; ii < m_vLightSources.GetSize(); ii++) 
    {
        SM_ASSERT(m_vLightSources[ii] != NULL);
        delete m_vLightSources[ii];
        m_vLightSources[ii] = NULL;
    }
    if (m_pRayTracer)
    {
        delete m_pRayTracer;
        m_pRayTracer = NULL; 
    }
} // end SmRayRenderer::~SmRayRenderer destructor

 
/*******************************************************************//**
PURPOSE: Find a color to assign to input ray based on what the
  ray intersects within the m_pRayTracer object.

NOTES: Fires input ray into the m_pRayTracer shape model, and

  1. when ray hits no surfaces - returns background color
  2. when ray hits something - init color to m_vDefaultAmbientColor
    2a. and for every light source
  
***********************************************************************/
SmStatus SmRayRenderer::RenderRay
  (const SmVector3d & crRayPoint,    // in : target Ray Start Point 
   const SmVector3d & crRayVector,   // in : target Ray Direction
   SmVector3d       & rRayColor)     // out: ray color s
{ 
  // locals
  SmSolution sSolution;
  SmBoolean bHitsSomething;
  
  // init state for upcoming Ray intersection
  m_pRayTracer->m_bHitAnyThing = FALSE; // Only take closest hit
  m_pRayTracer->m_pSkipElement = NULL;

  // intersect ray with shapes stored within m_pRayTracer
  SER(m_pRayTracer->FireRay(crRayPoint, crRayVector, 
                            SM_BIG_DOUBLE,
                            bHitsSomething, 
                            sSolution,
                            m_vGridElements, 
                            m_vMinDistances, 
                            m_vMaxDistances));
  
  // when nothing was hit - set output color to Background color
  if (!bHitsSomething) 
  {
      rRayColor = m_vBackgroundColor;
      return SM_SUCCESS;
  }

  // arrive here when input ray hits something

  // get local hit data
  m_lPixelsShaded ++;
  SmGridElement *pHitNode = (SmGridElement *)sSolution.m_apNodes[0];
  
  SmVector3d  sPoint  (  sSolution.m_vStart[3], 
                         sSolution.m_vStart[4],
                         sSolution.m_vStart[5]);
  SmVector3d sNormal  (  sSolution.m_vStart[6], 
                         sSolution.m_vStart[7],
                         sSolution.m_vStart[8]);

  // init output color to Ambient
  rRayColor = m_vDefaultAmbientColor;
  
  // for every light source
  ULONG ii;
  for (ii = 0; ii < m_vLightSources.GetSize(); ii++) 
    {
      SmLightSource *pLight = m_vLightSources[ii];

      // get angle between lightSource and hitNormal directions
      double dDot = pLight->m_vLightDirectionOrOrigin.Dot(sNormal);

      // when doing shadows - 
      // Trace a ray towards the light. 
      //   If it hits anything, this point is in shadow and
      //   won't get any of this light's color added to it.
      if (m_bDoShadows) 
        {
          SmVector3d sShadowVec = pLight->m_vLightDirectionOrOrigin;

          double dDotShadow = sShadowVec.Dot(sNormal);

          // If the angle between the shadow vector and the normal is near
          // 90 degrees then we need to shoot at the node we hit otherwise
          m_pRayTracer->m_pSkipElement = (smos_Fabs(dDotShadow) < 0.2) ? NULL : pHitNode;

          // start the ray just beyond the current hit point
          SmPoint3d sShadowPoint = sPoint + 0.0001 * sShadowVec;

          // save a little time - look for any hit not just the closest hit
          m_pRayTracer->m_bHitAnyThing = TRUE; 

          // shoot the ray to the light - see if anything else is hit
          SER(m_pRayTracer->FireRay(sShadowPoint, sShadowVec, 
                                    SM_BIG_DOUBLE, 
                                    bHitsSomething,
                                    sSolution, 
                                    m_vGridElements,
                                    m_vMinDistances, 
                                    m_vMaxDistances));

          //something was hit - set dDot to zero to remember this point is in shadow
          if (bHitsSomething) 
            {
              dDot = 0.0;
            }
        } // end doing shadows check

#ifdef SM_DEBUG_CODE
      ULONG lCount = rand() % 6;
      SmVector3d sDebugColor;
      if (lCount % 6 == 0) sDebugColor.Set(1, 0, 1);
      if (lCount % 6 == 1) sDebugColor.Set(0, 0, 1);
      if (lCount % 6 == 2) sDebugColor.Set(0, 1, 1);
      if (lCount % 6 == 3) sDebugColor.Set(1, 0, 0);
      if (lCount % 6 == 4) sDebugColor.Set(0, 1, 0);
      if (lCount % 6 == 5) sDebugColor.Set(1, 1, 0);

      // expect rays to only intersect forward facing surface normals
      if (sNormal.Dot(crRayVector) > 0.0) 
        { SE_MSG(SM_ERR,_T("ray hit back (not front) side of a face")); }
#endif

      // add Default Object Color to current ambient light
      if (dDot > 0.0) 
        {
          rRayColor = rRayColor + dDot * m_vDefaultObjectColor;
        }

    } // end iter every light source
  
  // Clamp colors to 1.0
  if (rRayColor.x > 1.0) rRayColor.x = 1.0;
  if (rRayColor.y > 1.0) rRayColor.y = 1.0;
  if (rRayColor.z > 1.0) rRayColor.z = 1.0;
  
  // all done
  return SM_SUCCESS;   
  
} // end SmRayRenderer::RenderRay

/*******************************************************************//**
PURPOSE: Render this pixel.

NOTES: Cast m_lSampleRate**2 rays from within the target pixel
   and determine a color for each depending on what it hits.

   Return the average of all sample ray colors as the output color.
***********************************************************************/
SmStatus SmRayRenderer::RenderPixel
  (ULONG        lPixelX,        // in : 
   ULONG        lPixelY,        // in : 
   SmVector3d & rPixelColor)    // out: 
{
  // init output
  rPixelColor.Set(0.0, 0.0, 0.0);

  // Get Ray direction
  SmVector3d sRayVector = m_vInvTransform.GetZAxis();
  
  // get lower corner parameter value for this pixel
  double dX =(lPixelX * 1.0) / m_lNumXPixels;
  double dY =(lPixelY * 1.0) / m_lNumYPixels;

  // figure the sample stepping size for one pixel
  // assuming all pixels sit in a 1x1 unit square.
  SmVector2d sSize;
  sSize.x = 1.0 /(m_lNumXPixels * m_lSampleRate);
  sSize.y = 1.0 /(m_lNumYPixels * m_lSampleRate);
  
  SmVector3d sRayColor;
  
  // for every sample point within the pixel
  ULONG ixs, iys;
  for (ixs = 0; ixs < m_lSampleRate; ixs++) 
    {
      for (iys = 0; iys < m_lSampleRate; iys++) 
        {
          // pick a pixel start point
          double dEvalX = dX + ixs * sSize.x;
          double dEvalY = dY + iys * sSize.y;
          SmVector3d sRayPoint = m_vViewVolume.Evaluate(dEvalX, dEvalY, 1.0);

          // fire ray into model and pick an associated color
          SER(RenderRay(sRayPoint, sRayVector, sRayColor));
          
          // accumulate sample point color values        
          rPixelColor = rPixelColor + sRayColor;
        }
    } // end iter every sample point within the pixel
  
  // set output color = average of all sample point colors
  rPixelColor = rPixelColor /(m_lSampleRate*m_lSampleRate);

  // all done
  return SM_SUCCESS;

} // end SmRayRenderer::RenderPixel


/*******************************************************************//**
PURPOSE: Render an image using raytracing.

NOTES: 
***********************************************************************/
SmStatus SmRayRenderer::DoRender
(SmBoolean bDoShadows,
   const SmVector3d & crBackgroundColor,
   const SmVector3d & crDefaultObjectColor)
{
    m_bDoShadows = bDoShadows;
    m_vBackgroundColor = crBackgroundColor;
    m_vDefaultObjectColor = crDefaultObjectColor;
    m_vDefaultAmbientColor = crDefaultObjectColor / 10.0;
    
    SmVector3d sPixelColor;
    SmTArray < SmVector3d> sScanLine(m_lNumXPixels, NULL, m_lNumXPixels);

    TCHAR sBuff[SM_TBLOCK_SIZE];
    SmBoolean bLastShaded = FALSE;
    SmBoolean bLastOff = FALSE;
    
    ULONG ix, iy;
    for (iy = 0; iy < m_lNumYPixels; iy++) 
    {
        for (ix = 0; ix < m_lNumXPixels; ix++) 
        {
            // Here we are working on pixel ix,iy as measured from
            // the upper left hand corner.
            SER(RenderPixel(ix, iy, sPixelColor));
            sScanLine[ix] = sPixelColor;
            if (sPixelColor.Length() == 0.0) 
            {
                if (bLastShaded)
                {
                    bLastOff = TRUE; 
                }
                else            { bLastOff = FALSE; }
                bLastShaded = FALSE;
            }
            else 
            {
                if (bLastOff) 
                {
                    //                    SE(SM_ERR);
                }
                bLastShaded = TRUE;
            }
        } 
        smos_sprintf(sBuff, _T("Rendered Scanline = %ld\n"), iy);
        smos_WriteBuffer(sBuff);
    }
    return SM_SUCCESS;
} // end SmRayRenderer::DoRender

#if 0
/*******************************************************************//**
PURPOSE:  Return Surface Pointer stored in 
 
               SmTreeNode->m_pTree                         type SmTree
               SmTreeNode->m_pTree->m_pOwner               type SmSurfaceCache
               SmTreeNode->m_pTree->m_pOwner->m_crSurface  type SmSurface

NOTES: UNUSED
***********************************************************************/
static const SmSurface & sm_GetOriginalSurface
(const SmTreeNode & crSurfaceNode)
{
  SmTree *pTree = crSurfaceNode.m_pTree;

  // now the same as: SmTreeNode::GetOwnerObject() ;
  // check state
  if (!pTree->GetOwner()->IsKindOf(SmSurfaceCache_TYPE)) 
    { SE(SM_ERR); }  // should never happen
  SmSurfaceCache *pSC =(SmSurfaceCache*)pTree->GetOwner();
  if (pSC == NULL) 
    { SE(SM_ERR); } // should never happen
  return pSC->GetSurface();

} // end sm_GetOriginalSurface
#endif  // 0

//      /*******************************************************************//**
//      PURPOSE: Convenience function to be used with the cached
//                  tree structure.  When the original surface is
//                  a BSplineSurface use the pBSS surface for evaluations.
//                  It is expected that pointer will be the Bezier surface
//                  patch stored in this part of the surface tree cache.
//      
//                  When the surface is not a BSplineSurface(it may be
//                  a SmOffsetSurface) the Bezier surface is approximate
//                  and the OriginalSurface should be used for evaluations.
//      NOTES:
//      ***********************************************************************/
//      static const SmSurface * sm_GetSurfaceToEvaluate
//      (const SmTreeNode * cpSurfaceNode,   // in : Node to query
//       const SmSurface  * pBSS)            // in : Node's bezier path
//      {
//        const SmSurface & rSurf = sm_GetOriginalSurface(*cpSurfaceNode);
//      
//        return &rSurf;  
//      } // end sm_GetSurfaceToEvaluate


/*******************************************************************//**
PURPOSE: Return TRUE when TestPoint is a valid SurfacePoint

NOTES: All code normally in SmTrimSrfCache::PointTest() was
   moved into this function to remove SmRayTrace dependencies on 
   SmSurfaceCache objects.
***********************************************************************/
static SmBoolean sm_IsPointValid
  (const SmSurface  * pSurface,        // in : Target Surface
   const SmPoint2d  & crUVPoint,       // in : test Surface Point
   SmZoneTol3d        sSrcZoneTol3d,   // in : ObjZoneTol3d assoc with UVPoint for PointClassify() call
                                       //      if none, use: SmTol::GetZoneTol3d(BREP_CONTEXT_OR_NULL)
   SmObject        *& pObject)         // out: pointer to topology object coincident with point
{
  // init output
  pObject = NULL;

  // locals
  SmBoolean rbPointIsOK ;
  SmFace    * pFace = (SmFace *)pSurface->GetFace() ;
  SmZoneTol3d sFaceZoneTol3d = SmTol::GetZoneTol3d(pFace) ;

  // no face - check crUVPoint agains Natural Domain
  if(pFace == NULL) 
    { 
      rbPointIsOK = pSurface->GetNaturalUVDomain().ContainsPoint2d(crUVPoint) ; 
    }
  else // face
    {
      // classify point (with default tolerance) against face
      SmPointClassification sPC(sFaceZoneTol3d, pSurface->GetContext()) ; // should be crUVPoint ZoneTol3d

      sPC.SetSrcZoneTol3d( sSrcZoneTol3d );

      SmStatus sStatus = pFace->PointClassify(crUVPoint, sSrcZoneTol3d, FALSE, TRUE, sPC) ;

      if (sStatus != SM_SUCCESS) 
        {
          // Signal errors from the Face->PointClassify() call
          //   and tell the caller to keep the point just in case.
          SE(SM_ERR);
          SM_DBG_WARN(_T("SmSurface.cpp sm_IsPointValid() - passed a point that caused an error in SmFace::PointClassify()"));
          return TRUE;
        }

      // Point is ok when it classified to a topology object
      rbPointIsOK =  (   sPC.GetPointClass() == SM_PC_FACE
                      || sPC.GetPointClass() == SM_PC_EDGE
                      || sPC.GetPointClass() == SM_PC_VERTEX) 
                    ? TRUE
                    : FALSE ;
      pObject = sPC.GetObject() ;
    } 

  // all done
  return(rbPointIsOK) ;

} // end sm_IsPointValid

// GWC:Obsolete method with SmSurfaceCache and SmTrimSrfCache dependencies
//      /*******************************************************************//**
//      PURPOSE: Return TRUE when TestPoint is a valid SurfacePoint
//      
//      NOTES: When Surface Cache is type
//         SmSurfaceCache_TYPE, return TRUE when TestPoint is with Surface Domain
//         SmTrimSrfCache_TYPE, return TRUE when testPoint is within Surface Trim Boundaries
//      ***********************************************************************/
//      static SmBoolean sm_IsPointValid
//        (const SmTreeNode & crSurfaceNode,   // in : Target Surface Node
//         const SmPoint2d  & crUVPoint,       // in : test Surface Point
//         SmObject        *& pObject)         // out: pointer to topology object coincident with point
//      {
//        // init output
//        pObject = NULL;
//        
//        // locals
//        SmTree         *pTree = crSurfaceNode.m_pTree;
//        SmSurfaceCache *pSC   =(SmSurfaceCache*)pTree->GetOwner();
//        
//        // keep all points when SurfaceNode is not associated with some kind of SurfaceCache
//        if (!pTree->GetOwner()->IsKindOf(SmSurfaceCache_TYPE)) 
//          {
//            SE(SM_ERR);
//            return TRUE;    
//          }
//        
//        // Test Point against surface->Owner TrimBoundaries 
//        //   when pSC is a TrimSrfCache (not a SurfaceCache)
//        //            is current
//        //        and pSC->m_bPointTestEnabled == TRUE
//        SmBoolean bRet    = TRUE;
//        SmStatus  sStatus = pSC->PointTest(crUVPoint, bRet, NULL, &pObject);
//        if (sStatus != SM_SUCCESS) 
//        {
//            // Signal errors from the Face->PointClassify() call
//            //   and tell the caller to keep the point just in case.
//            SE(SM_ERR);
//            SM_DBG_WARN(_T("SmSurface.cpp sm_IsPointValid() - passed a point that caused an error in SmFace::PointClassify()"));
//            return TRUE;
//        }
//        
//        // all done
//        return bRet;
//      
//      } // end sm_IsPointValid

/*******************************************************************//**
PURPOSE: Given a tangent vector and delta vector - determine the    
            parameter delta which would move the evaluation the correct
            distance such that the projection of the delta vector would
            fall near the new point.                                   
NOTES:
***********************************************************************/
static double sm_ConvertToParameterDelta
(const SmVector3d & crTangentVector,
   const SmVector3d & crDeltaVector)
{
    double dTanLen = crTangentVector.Length();
    if (dTanLen < SM_EFF_ZERO)
        return 0.0;
    SmVector3d sUnitTan = crTangentVector / dTanLen;
    double dDot = sUnitTan.Dot(crDeltaVector);
    return dDot/dTanLen;
} // end sm_ConvertToParameterDelta

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
static SmStatus sm_FindAdditionalTangentGuess
  (const SmSurface  & crSurface,
   const SmExtent2d & crUVDomain,
   const SmCurve    & crCurve,
   const SmExtent1d & crInterval,
   const SmPoint2d  & crCurrentUV,
   double dCurrentT,
   SmBoolean & rbFoundGuess,
   SmPoint2d & rNewUVGuess,
   double & rdNewTGuess)
{
    rbFoundGuess = FALSE;
    
    SmVector3d sNormal;
    SER(crSurface.EvaluateNormal(crCurrentUV, TRUE, TRUE, sNormal));
    SmVector3d sPV[2];
    SER(crCurve.Evaluate(dCurrentT, 1, TRUE, sPV));
    // First do a quick test to determin the angle of 
    // intersection between the tangent plane of the surface
    // ane the tangent of the curve.  If not within 20 degrees
    // assume that we don't have a tangency touching situation.
    if (!sNormal.IsPerpendicularTo(sPV[1], 20.0)) 
    { return SM_SUCCESS; }
    // Lets take a normal section on the surface along the direction
    // of the tangent of the curve.  This should give us two circles
    // or a line and a circle to intersect.
    double dNormalCurvature = 0.0;
    SmVector3d sTanPlaneProj;
    SER(crSurface.EvaluateNormalSection(crCurrentUV, TRUE, TRUE,
        sPV[1], sTanPlaneProj, dNormalCurvature, sNormal));
    
    // First compute the projected tangent vector.
    SmPoint3d sSurfPoint;
    SER(crSurface.EvaluatePoint(crCurrentUV, sSurfPoint));
    
    // Get the geometric properties of the curve at the intersection
    SmVector3d sGeomVec[4];
    SER(crCurve.EvaluateGeometric(dCurrentT, 2, TRUE, sGeomVec));
    SmVector3d sCurvatureVector = sGeomVec[2];
    double dCurveCurvature = sCurvatureVector.Length();
    if (dCurveCurvature > SM_EFF_ZERO)
        SER(sCurvatureVector.Unitize());
    
    // Compute the normal to the plane which is defined by
    // the surface normal and the projected vector.
    SmVector3d sProjectionPlaneNormal = sTanPlaneProj * sNormal;
    SER(sProjectionPlaneNormal.Unitize());
    
    // If the normal curvature is zero then use a line instead of 
    // a circle for the surface.
    // Also if the radius of one circle is greater then twice the 
    // other use a line for the one with the larger radius.
    SmBoolean bLineCircleIntersection = FALSE;
    double dCircle1Radius, dCircle2Radius=0.0;
    SmPoint3d sLinePoint, sCircle1Center, sCircle2Center;
    SmVector3d sLineVec, sCircle1Normal, sCircle2Normal;
    SmVector3d sFoundVector;
    if (smos_Fabs(dNormalCurvature) < SM_EFF_ZERO) 
    {
        // If the curve also has zero curvature then we are done
        if (smos_Fabs(dCurveCurvature) < SM_EFF_ZERO) 
        { return SM_SUCCESS; }   // Done with no new guesses needed
        dCircle1Radius = smos_Fabs(1.0/dCurveCurvature);
        sCircle1Center = sPV[0] + sCurvatureVector * dCircle1Radius;
        sCircle1Normal = sPV[1] * sCurvatureVector;
        if (sCircle1Normal.LengthSquared() < SM_EFF_ZERO_SQ) 
        { return SM_SUCCESS; }
        SER(sCircle1Normal.Unitize());
        sLinePoint = sSurfPoint;
        sLineVec = sTanPlaneProj;
        SER(sLineVec.Unitize());
        bLineCircleIntersection = TRUE;
    }
    else 
    { // have surface with curvature
        dCircle1Radius = smos_Fabs(1.0 / dNormalCurvature);
        SER(sNormal.Unitize());
        sCircle1Center = sSurfPoint + sNormal *(1.0 / dNormalCurvature);
        sCircle1Normal = sProjectionPlaneNormal;
        if (smos_Fabs(dCurveCurvature) < SM_EFF_ZERO  
            || smos_Fabs(1.0/dCurveCurvature) > 2.0 * dCircle1Radius) 
        {
            sLinePoint = sPV[0];
            sLineVec = sPV[1];
            SER(sLineVec.Unitize());
            bLineCircleIntersection = TRUE;
        }
        else 
        {
            dCircle2Radius = smos_Fabs(1.0/dCurveCurvature);
            if (dCircle1Radius > 2.0 * dCircle2Radius) 
            {
                dCircle1Radius = dCircle2Radius;
                sCircle1Center = sPV[0] + sCurvatureVector * dCircle2Radius;
                sCircle2Normal = sPV[1] * sCurvatureVector;
                if (sCircle2Normal.LengthSquared() < SM_EFF_ZERO_SQ) 
                { return SM_SUCCESS; }
                sLinePoint = sSurfPoint;
                sLineVec = sTanPlaneProj;
                SER(sLineVec.Unitize());
                bLineCircleIntersection = TRUE;
            }
            else 
            { // nearly equal radii
                SER(sCurvatureVector.Unitize());
                sCircle2Center = sPV[0] + sCurvatureVector * dCircle2Radius;
                sCircle2Normal = sPV[1] * sCurvatureVector;
                if (sCircle2Normal.LengthSquared() < SM_EFF_ZERO_SQ) 
                { return SM_SUCCESS; }
            }
        }
    }
    
    if (bLineCircleIntersection) 
    {
        ULONG lNumTsect;
        double adTsectLineParam[2];
        SER(smgu_LineCoPlanarCircleIntersect(sLinePoint, sLineVec,
            sCircle1Center, sCircle1Normal,
            dCircle1Radius, SM_EFF_ZERO,
            lNumTsect,    adTsectLineParam));
        // If only have one or zero intersections we are done without
        // picking up extra guess
        if (lNumTsect != 2) 
        { return SM_SUCCESS;   }
        // Find the other intersection -- i.e. the one we don't have
        double dLineT = adTsectLineParam[0];
        if (smos_Fabs(adTsectLineParam[1]) > smos_Fabs(adTsectLineParam[0])) 
        {
            dLineT = adTsectLineParam[1];
        }
        SmPoint3d sOtherTsect = sLinePoint + dLineT * sLineVec;
        sFoundVector = dLineT * sLineVec;
    }
    else 
    { // Do circle circle intersection
        ULONG lNumTsect;
        SmPoint3d sTsectPoints[2];
        if (dCircle1Radius > dCircle2Radius) 
        {
            SER(smgu_CircleCoPlanarCircleIntersect(sCircle1Normal,
                sCircle1Center, dCircle1Radius, sCircle2Center, dCircle2Radius,
                SM_EFF_ZERO,
                lNumTsect, sTsectPoints));
        }
        else 
        {
            SER(smgu_CircleCoPlanarCircleIntersect(sCircle2Normal,
                sCircle2Center, dCircle2Radius, sCircle1Center, dCircle1Radius,
                SM_EFF_ZERO,
                lNumTsect, sTsectPoints));
        }
        // If only have one or zero intersections we are done without
        // picking up extra guess
        if (lNumTsect != 2)
            return SM_SUCCESS;
        sFoundVector = sTsectPoints[0] - sSurfPoint;
        if (sFoundVector.Length() < sTsectPoints[1].DistanceBetween(sSurfPoint)) 
        {
            sFoundVector = sTsectPoints[1] - sSurfPoint;
        }
    }
    
    // Use the found vector to compute an approximate step in the
    // curve and surface parameters.
    SmVector3d sDU, sDV;
    SER(crSurface.Evaluate1stDerivatives(crCurrentUV, TRUE, TRUE, sSurfPoint, sDU, sDV));
    
    rbFoundGuess = TRUE;
    rdNewTGuess = dCurrentT + sm_ConvertToParameterDelta(sPV[1], sFoundVector);
    rNewUVGuess.x = crCurrentUV.x + sm_ConvertToParameterDelta(sDU, sFoundVector);
    rNewUVGuess.y = crCurrentUV.y + sm_ConvertToParameterDelta(sDV, sFoundVector);
    
    if (!crInterval.ContainsValue(rdNewTGuess) 
        && !crUVDomain.ContainsPoint2d(rNewUVGuess)) 
    {
        // If the guess is outside of the domain we'll be able to pick it up using
        // another method.
        rbFoundGuess = FALSE;
        return SM_SUCCESS;
    }
    rdNewTGuess = crInterval.ClampValue(rdNewTGuess);
    rNewUVGuess = crUVDomain.ClampPoint2d(rNewUVGuess);
    
#ifdef SM_DEBUG_CODE
    SmBoolean bDebugMe = FALSE;
    if (bDebugMe) 
    {
        sm_GraphicsLoop();
        smgfx_SetColor(0, 0, 0);
        crCurve.DrawAt(dCurrentT, 0);
        smgfx_SetColor(1, 0, 0);
        crCurve.DrawAt(rdNewTGuess, 0);
        smgfx_SetColor(0, 0, 1);
        crSurface.DrawAt(rNewUVGuess, 0);
        sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE
    
    return SM_SUCCESS;  // Found a guess - loaded it now we are done
} // end sm_FindAdditionalTangentGuess




/*******************************************************************//**
PURPOSE: Constructor for the SmRayTracer object.  

NOTES: 
***********************************************************************/
SmRayTracer::SmRayTracer
(const SmContext & crContext,
   SmGrid * pGrid,
   double dRayTolerance,
   double dRayAccuracy)
 : m_crContext(crContext),
   m_dRayTolerance(dRayTolerance), 
   m_dRayAccuracy(dRayAccuracy),
   m_pGrid(pGrid)  
{
    m_sLSISolver.m_sUVDomain          = SmExtent2d(SmPoint2d(-SM_BIG_DOUBLE, -SM_BIG_DOUBLE),
                                                   SmPoint2d( SM_BIG_DOUBLE,  SM_BIG_DOUBLE));
    m_sLSISolver.m_bHaveTMinTMax      = TRUE;
    m_sLSISolver.m_bRayFire           = TRUE;
    m_sLSISolver.m_dDistanceTolerance = dRayTolerance;
    m_sLSISolver.m_dRayAccuracy       = dRayAccuracy;
    m_sLSISolver.m_sLineInterval.SetMinMax(0.0, 1.0E10) ;

} // end SmRayTracer::SmRayTracer constructor

/*******************************************************************//**
PURPOSE: Destructor for the SmRayTracer object

NOTES: It destroys the grid but not the surfaces.
***********************************************************************/
SmRayTracer::~SmRayTracer()
{
  if(m_pGrid) 
    { delete m_pGrid ; m_pGrid = NULL; }

} // end SmRayTracer::~SmRayTracer destructor


/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
SmLSI3LocalSolver::SmLSI3LocalSolver
 (SmTree           * pSurfaceTree,        // NotUsed: in : 
  const SmExtent2d & crUVDomain,          // in : 
  const SmPoint3d  & crLinePoint,         // in : 
  const SmVector3d & crLineVector,        // in : 
  const SmExtent1d * cpLineInterval,      // in : 
  SmBoolean          bRayFire,            // in : 
  double             dDistanceTolerance,  // in : 
  double             dRayAccuracy)        // in : 
 :
  m_d3dTolerance(0.0),
  m_sUVDomain(crUVDomain),
  m_vLinePoint(crLinePoint),
  m_vLineVector(crLineVector),
  m_sLineInterval(0.0,1.0e10),
  m_bRayFire(bRayFire),
  m_dDistanceTolerance(dDistanceTolerance),
  m_dRayAccuracy(dRayAccuracy),
  m_bHaveTMinTMax(FALSE)
{
  SM_REF1(pSurfaceTree) ; 
  if(cpLineInterval)
    {
      m_sLineInterval = *cpLineInterval ;
    } 
} // end SmLSI3LocalSolver::SmLSI3LocalSolver


/*******************************************************************//**
PURPOSE:

NOTES: This is the Solver called by The SmRayTracer::FireRay() method
  when the SmRayTracer->SmGrid is loaded with SmBrep->SmSurfaces.
  (The Grid could be loaded with SmPolyBreps)
***********************************************************************/
SmStatus SmLSI3LocalSolver::LocalSolve
  (SmSurfaceGridElement * pSurfaceGridElement,  // in : Surface Decomposition TreeNode Summary data
   SmBoolean  & rbNeedsMoreSubdivision,         // out: Always set to FALSE
   SmBoolean  & rbFoundIntersection,            // out: TRUE = Valid Point was found
   SmSolution & rSolution)                      // out: When rbFoundIntersection == TRUE contains
                                                //      Ray/SurfaceNode intersection data as:
                                                //       rSolution.m_vStart[0] = T parameter of line                  
                                                //       rSolution.m_vStart[1] = U of UVPoint          
                                                //       rSolution.m_vStart[2] = V of UVPoint                              
                                                //       rSolution.m_vStart[3] = X of 3D point  
                                                //       rSolution.m_vStart[4] = Y of 3D point                              
                                                //       rSolution.m_vStart[5] = Z of 3D point                              
                                                //       rSolution.m_vStart[6] = n.X of 3D Normal Vector       
                                                //       rSolution.m_vStart[7] = n.Y of 3D Normal Vector                              
                                                //       rSolution.m_vStart[8] = n.Z of 3D Normal Vector                              
                                                //       rSolution.m_apNodes[0]   =  
                                                //       rSolution.m_apObjects[0] = Surface Pointer
                                                //       rSolution.m_apObjects[0] = Face Pointer or NULL
{
  // init output
  rbFoundIntersection    = FALSE ;
  rbNeedsMoreSubdivision = FALSE ;
  
  // locals
  SM_ASSERT(pSurfaceGridElement->GetType() == SmSurfaceGridElement_TYPE) ;
  const SmSurface *pSurface     =  pSurfaceGridElement->m_pSurface ;
  SmVector3d      *pMidPoint    = &pSurfaceGridElement->m_sMidPoint ; 
  SmVector3d      *pMidDU       = &pSurfaceGridElement->m_sMidDU ;    
  SmVector3d      *pMidDV       = &pSurfaceGridElement->m_sMidDV ;    
  SmVector3d      &sMidNormal   =  pSurfaceGridElement->m_sMidNormal ;

  // get patch UVDomain, regular BBox, PolarBox and PseudoBox  
  const SmExtent2d & rUVDomain   =  pSurfaceGridElement->m_sUVDomain ;
  SmPolarBox       & rPolarBox   =  pSurfaceGridElement->m_sPolarBox ;
  SmPseudoBox        sPseudoBox  =  pSurfaceGridElement->m_sPseudoBox;
  sPseudoBox.ExpandAbsolute(m_dDistanceTolerance);

  SmPoint3d s3DPoint;
  SmPoint2d sUVPointDelta(0, 0);
  double    dLineParam;
  
  // Get PatchMidNormal/LineVector dot product - to measure angle
  // gwc:SHORTCACHE double dDotNorm = m_vLineVector.Dot(pBezPatch->m_sMidNormal);
  double dDotNorm = m_vLineVector.Dot(sMidNormal);
  
  // when PatchMidPoint/LineVector is large( more than 60) - use Corner Guess Points
  SmBoolean bUseCorners =(smos_Fabs(dDotNorm) < 0.4)
      ? TRUE
      : FALSE;
  // get ray/MidTangentPlane intersection
  if (SM_SUCCESS != sm_LinePlaneIntersect(m_vLinePoint, m_vLineVector,              // the line
                                            *pMidPoint,    sMidNormal,                 // the plane
                                            *pMidDU,      *pMidDV,                     // the plane
                                             s3DPoint,     sUVPointDelta, dLineParam)) // the intersections
    {
      bUseCorners = TRUE;
    }
  
  // when using corners - let 3DGuessPoint = Closest LinePoint to patchMidPoint
  if (bUseCorners) 
    {
      SER(smgu_LineClosestPoint(m_vLinePoint, m_vLineVector,
                                *pMidPoint,
                                dLineParam));
      s3DPoint = m_vLinePoint + dLineParam * m_vLineVector;
    }
  
#ifdef SM_DEBUG_CODE
  SmExtent3d* pBBox = &pSurfaceGridElement->m_sBBox;

  SmBoolean bDebugMe = FALSE;
  if (bDebugMe) 
    {
      SmVector3d sVec = dLineParam * m_vLineVector;
      SmPoint3d sPnt = m_vLinePoint + sVec;
      SmPlane sPlane(*pMidPoint, sMidNormal, pSurface->GetContext()) ;
      
      smgfx_Erase();
      smgfx_SetLook(3,4, 1, 0, 0); m_vLinePoint.Draw(); sm_GraphicsLoop();
      smgfx_SetLook(1,2, 0, 0, 1); pSurface->DrawUV(8, 8); sm_GraphicsLoop();
      smgfx_SetLook(3,4, 0, 0, 1); s3DPoint.Draw(); sm_GraphicsLoop();
      smgfx_SetLook(1,2, 0, 0, 1); pBBox->Draw(NULL); sm_GraphicsLoop();
      smgfx_SetLook(3,4, 0, 0, 1); sVec.Draw(&m_vLinePoint); sm_GraphicsLoop();
      smgfx_SetLook(6,7, 0, 0, 1); sPnt.Draw(); sm_GraphicsLoop();
      smgfx_SetLook(1,2, 0, 1, 0); sPlane.Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 1, 1, 0); sPseudoBox.Draw(NULL); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif

  // when the 3DGuessPoint is not in the pseudoBox
  // try to eliminate intersection possibility by looking 
  //  at the psuedoBoxPlane/Line intersections and
  //  quitting when no intersection can be found inside
  //  both the bounding and pseudo box of the node.

  if (!sPseudoBox.ContainsPoint3d(s3DPoint)) 
    {
      // If the point is not in the pseudo box, check whether the ray intersects it anywhere.
      ULONG lNumInts=0;
      double dT0 = 0.0, dT1 = 0.0;
      sPseudoBox.IntersectLine( m_vLinePoint, m_vLineVector, lNumInts, dT0, dT1 );

      // If no intersection found just return here
      if ( lNumInts == 0 )
        {
          return SM_SUCCESS;
        }
    } // end 3dGuessPoint is not contained by PatchPseudoBox check
  
  // Arrive here when we need to look for a line/patch intersection using numerical methods.
  {
      // locals
      const SmContext * pContext    = pSurface->GetContext();
      double            d3DAccuracy =   (m_dRayAccuracy <= SM_EFF_ZERO/10.0)
                                      ? SM_EFF_ZERO * 100.0 *(1.0 + s3DPoint.GetMaxDimension())
                                      : m_dRayAccuracy;
      NER(pContext);
      
#ifdef SM_DEBUG_CODE
      SmBoolean bDebugMe4 = FALSE;
      if (bDebugMe4) 
        {
          SmVector3d sLineVec = 30 * m_vLineVector;
          
          smgfx_Erase();
          smgfx_SetLook(3,4, 1, 0, 0) ; m_vLinePoint.Draw(); sm_GraphicsLoop();
          smgfx_SetLook(2,3, 1, 0, 1) ; sLineVec.Draw(&m_vLinePoint); sm_GraphicsLoop();
          smgfx_SetLook(1,2, 0, 0, 1) ; pSurface->DrawUV(3, 3, FALSE, &rUVDomain); sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif
      
      // Do a quick test right here to see if we can possibly 
      // eliminate needing to do more intersections

      // Get Line/PatchMidPoint intersection UVPoint in patch coordinates
      SmPoint2d sUVPoint;
      sUVPoint = rUVDomain.Evaluate(0.5, 0.5) + sUVPointDelta;

      // Omit the following test.  It seems reasonable, but it can reject good hits. [B607]
      // It is never hit in prog_test.
      //// when using the line/PatchMidPointTangencyPlane intersection as the GuessPoint
      //if (!bUseCorners)
      //  {
      //    // When the plane GuessPoint is not in the Patch Domain
      //    if (!rUVDomain.ContainsPoint2d(sUVPoint)) 
      //      {
      //        SmVector2d  sUVTol = rUVDomain.GetSize() / 100.0;
      //        SmBoolean bUOutMin = (sUVPoint.x < rUVDomain.GetMin().x) ;
      //        SmBoolean bUOutMax = (sUVPoint.x > rUVDomain.GetMax().x) ;
      //        SmBoolean bVOutMin = (sUVPoint.y < rUVDomain.GetMin().y) ;
      //        SmBoolean bVOutMax = (sUVPoint.y > rUVDomain.GetMax().y) ;
      //
      //        // Get Clamped UVpoint PatchPoint and Normal Vector
      //        sUVPoint = rUVDomain.ClampPoint2d(sUVPoint);
      //        SmVector3d sPVVV[2][2];
      //        SER(pSurface->Evaluate(sUVPoint, 1, 1, TRUE, TRUE, TRUE, sPVVV[0]));
      //        SmVector3d & rDV = sPVVV[0][1];
      //        SmVector3d & rDU = sPVVV[1][0];
      //        SmVector3d sNormal = rDU * rDV;
      //        if (sNormal.LengthSquared() < SM_EFF_ZERO_SQ) 
      //          {
      //            SER(pSurface->EvaluateNormal(sUVPoint, TRUE, TRUE, sNormal));
      //          }
      //        SmVector3d & rSrfPnt = sPVVV[0][0];
      //
      //        // Intersect Line with TangencyPlane at this new patchPoint
      //        if (sm_LinePlaneIntersect(m_vLinePoint, m_vLineVector,
      //                                  rSrfPnt, sNormal, rDU, rDV,
      //                                  s3DPoint, sUVPointDelta, dLineParam) != SM_SUCCESS) 
      //          { return SM_SUCCESS; }
      //
      //        // if this intersection point is out of bounds - no intersection
      //        sUVPoint = sUVPoint + sUVPointDelta;
      //        if (rUVDomain.ContainsPoint2d(sUVPoint, SM_EFF_ZERO_SQRT) == FALSE) 
      //          {
      //            if (   (sUVPoint.x < rUVDomain.GetMin().x - sUVTol.x && bUOutMin == TRUE)
      //                || (sUVPoint.x > rUVDomain.GetMax().x + sUVTol.x && bUOutMax == TRUE) 
      //                || (sUVPoint.y < rUVDomain.GetMin().y - sUVTol.y && bVOutMin == TRUE)
      //                || (sUVPoint.y > rUVDomain.GetMax().y + sUVTol.y && bVOutMax == TRUE)) 
      //              { return SM_SUCCESS; } 
      //          }
      //
      //        // set GuessPoint with clamped point value
      //        sUVPoint = rUVDomain.ClampPoint2d(sUVPoint);
      //      } // end GuessPoint not in patch UVDomain check
      //  } // when GuessPoint is the patchMidPoint check
      
      // arrive here when Line/Patch intersection is likely
      
      // set ray interval
      SmExtent1d sTmpIvl(-1.0e10, 1.0e10);

      if ( ! m_sLineInterval.IsInit() )  // (Note, IsInit() means uninitialized.)
        {
          sTmpIvl = m_sLineInterval;
        }
      
      // NR Solver locals
      double     dMinTFound         = SM_BIG_DOUBLE;
      SmBoolean  bFoundIntersection = TRUE;
      SmPoint2d  sUVFound(0.59087, 0.5654);
      double     dTFound            = 0.1;
      double     dDeviation         = 0.000001;
      SmPoint3d  sPnt(0, 0, 0);
      SmVector3d sNormal(0, 0, 1);
      
      // Add the first GuessPoint to the guess Point array
      double     dUVData[64];
      SmTArray < double> sUVPoints(64, dUVData);
      sUVPoints.Add(sUVPoint.x);
      sUVPoints.Add(sUVPoint.y);
      ULONG lTotalPoints = 1;
      
      // when needed - add in the corner points to the guess point array
      if (bUseCorners) 
        {
          sUVPoint = rUVDomain.Evaluate(0, 0);
          sUVPoints.Add(sUVPoint.x);
          sUVPoints.Add(sUVPoint.y);
          sUVPoint = rUVDomain.Evaluate(1, 0);
          sUVPoints.Add(sUVPoint.x);
          sUVPoints.Add(sUVPoint.y);
          sUVPoint = rUVDomain.Evaluate(0, 1);
          sUVPoints.Add(sUVPoint.x);
          sUVPoints.Add(sUVPoint.y);
          sUVPoint = rUVDomain.Evaluate(1, 1);
          sUVPoints.Add(sUVPoint.x);
          sUVPoints.Add(sUVPoint.y);
          lTotalPoints = 5;
        }
      
      ULONG lOrigPoints = lTotalPoints;
      
      // For every guess point - call the intersector to seek a solution.
      ULONG ii;
      for (ii = 0; ii < lTotalPoints; ii++) 
        {
          sUVPoint.x = sUVPoints[ii*2];
          sUVPoint.y = sUVPoints[ii*2 + 1];

          SER( pSurface->LocalLineIntersect(
                  rUVDomain,
                  m_vLinePoint,
                  m_vLineVector,
                  sTmpIvl,
                  m_d3dTolerance,
                  d3DAccuracy,
                  sUVPoint,

                  bFoundIntersection,
                  sUVFound,
                  dTFound,
                  dDeviation,
                  sPnt,
                  sNormal));

          // skip to next guess when no intersection was found
          if (!bFoundIntersection)
            { continue; }
          
          // save solution with lowest in-range ray parameter value
          if(   dTFound < dMinTFound
             && m_sLineInterval.ContainsValue(dTFound)) 
            {
#ifdef SM_DEBUG_CODE
              if (bDebugMe) 
                {
                  SmPoint3d sStartPnt ;
                  SmPoint3d sResultPnt = m_vLinePoint + dTFound * m_vLineVector;
                  SmStatus  sRtn = pSurface->EvaluatePoint(sUVPoint, sStartPnt) ;
                  
                  smgfx_Erase();
                  smgfx_SetLook(3,4, 0,1,0) ; m_vLinePoint.Draw(); sm_GraphicsLoop();
                  smgfx_SetLook(2,3, 0,1,0) ; (30 * m_vLineVector).Draw(&m_vLinePoint); sm_GraphicsLoop();
                  smgfx_SetLook(4,5, 1,0,0) ; sResultPnt.Draw(); sm_GraphicsLoop();
                  smgfx_SetLook(2,3, 0,1,1) ; if(sRtn == SM_SUCCESS) sStartPnt.Draw() ; sm_GraphicsLoop();
                  smgfx_SetLook(1,2, 0,0,1) ; pSurface->DrawUV(3, 3, FALSE, &rUVDomain); sm_GraphicsLoop();
                  sm_GraphicsLoop();
                }
#endif // SM_DEBUG_CODE
              // see if point is valid (within surface trim boundaries)
              SmObject *pObject = NULL;

              // This validation can be done with or without a surface cache
              //  GWC:This file's version of sm_IsPointValid() was rewritten
              //      to be independent of SmSurfaceCache and SmTrimSrfCache.
              //  GWC:removed 1 line: if (sm_IsPointValid(*pSurfaceNode, sUVFound, pObject)) 
              if (sm_IsPointValid(pSurface, sUVFound, m_d3dTolerance, pObject))
                {
                  // save the solution
                  rbFoundIntersection = TRUE;
                  dMinTFound                = dTFound;
                  rSolution.m_lNumObjects   = 1;
                  rSolution.m_apObjects[0]  = (SmSurface *)pSurface ;
                  rSolution.m_apObjects[1]  = pObject;
                  rSolution.m_lNumVariables = 9;
                  rSolution.m_vStart[0] = dTFound;
                  rSolution.m_vStart[1] = sUVFound.x;
                  rSolution.m_vStart[2] = sUVFound.y;
                  rSolution.m_vStart[3] = sPnt.x;
                  rSolution.m_vStart[4] = sPnt.y;
                  rSolution.m_vStart[5] = sPnt.z;
                  rSolution.m_vStart[6] = sNormal.x;
                  rSolution.m_vStart[7] = sNormal.y;
                  rSolution.m_vStart[8] = sNormal.z;
                  rSolution.m_vStart.m_dSolutionValue = dDeviation;
                  rSolution.m_eSolutionType           = SM_ST_SINGLE_VALUE;
                  // gwc: evil-evil hack - illegal type cast just to get SHORTRAYTRACER changes complete
                  //      this only works because of spaghetti code. 
                  //      This value only gets checked within SmRayTracer::FireRay where
                  //      it is cast back to its proper type prior to use.  yuk!
                  rSolution.m_apNodes[0]              = (SmTreeNode *)pSurfaceGridElement ;  // Surface node
  
                } // end point is valid (within trim boundaries) check
            } // end solution with smaller ray parameter found check
          
          // Try to get done right here.  If we have an intersection
          // then we can look at the vector map of the surface and
          // try to eliminate the possibility of additional intersections.
          SmVector3d sLineVec = m_vLineVector;
          double dAngle;

          // If the angle is less than 45 degrees than we can quit here
          // with out additional work to test for more tangent guesses.
          SER(sLineVec.AngleBetween(sNormal, dAngle));
          if (dAngle > SM_PI/2.0)
              dAngle = SM_PI - dAngle;
          if (dAngle < 60.0 * SM_PI / 180.0) 
            { return SM_SUCCESS; }

          SER(sLineVec.Unitize());
          if (!rPolarBox.HasPerpendicularToVector(sLineVec)) 
            { return SM_SUCCESS; }

          // If we have found an intersection look closely at it
          // and see if there might be another intersection very
          // close to it.  If there is compute a guess point for
          // that other intersection and add it to the list of guesses.
          // This should pick up any near tangent cases - like ray
          // firing just inside of a silhouette.
          if (ii < lOrigPoints) 
            {
              SmPoint2d sNewUVGuess;
              double dNewTGuess;
              SmBoolean bFoundGuess;
              SmLine sTmpLine(m_vLinePoint, m_vLineVector, 3, FALSE, pContext);
              SER(sm_FindAdditionalTangentGuess(*pSurface, rUVDomain, sTmpLine, sTmpIvl,
                                                   sUVFound, dTFound, bFoundGuess, 
                                                   sNewUVGuess, dNewTGuess));
              if (bFoundGuess) 
                {
                  lTotalPoints ++;
                  sUVPoints.Add(sNewUVGuess.x);
                  sUVPoints.Add(sNewUVGuess.y);

#ifdef SM_DEBUG_CODE
              if (bDebugMe) 
                {
                  SmPoint3d sNewStartPnt ;
                  SmPoint3d sResultPnt = m_vLinePoint + dTFound * m_vLineVector;
                  SmStatus  sRtn = pSurface->EvaluatePoint(sNewUVGuess, sNewStartPnt) ;
                  SmPoint3d sNewRayPoint = m_vLinePoint + dNewTGuess * m_vLineVector ;
                  //double dNewGap = (sNewRayPoint - sNewStartPnt).Length() ;
                  
                  smgfx_Erase();
                  smgfx_SetLook(3,4, 0,1,0) ; m_vLinePoint.Draw(); sm_GraphicsLoop();
                  smgfx_SetLook(2,3, 0,1,0) ; (30 * m_vLineVector).Draw(&m_vLinePoint); sm_GraphicsLoop();
                  smgfx_SetLook(4,5, 1,0,0) ; sResultPnt.Draw(); sm_GraphicsLoop();
                  smgfx_SetLook(5,6, 0,1,1) ; if(sRtn == SM_SUCCESS) sNewStartPnt.Draw() ; sm_GraphicsLoop();
                  smgfx_SetLook(5,6, 1,0,1) ; if(sRtn == SM_SUCCESS) sNewRayPoint.Draw() ; sm_GraphicsLoop();
                  smgfx_SetLook(1,2, 0,0,1) ; pSurface->DrawUV(3, 3, FALSE, &rUVDomain); sm_GraphicsLoop();
                  sm_GraphicsLoop();
                }
#endif // SM_DEBUG_CODE

                }
            }
        } // end iter every guess point
    } // end call the intersector to find intersection points block
  
  // all done
  return SM_SUCCESS;

} // end SmLSI3LocalSolver::LocalSolve


/*******************************************************************//**
PURPOSE: Intersect ray with input node and its parents

NOTES:
***********************************************************************/
SmStatus sm_FindPolyHit
  (SmTreeNode       * pNode,          // in : Target node
   const SmPoint3d  & crRayPoint,     // in : Ray Start Point
   const SmVector3d & crRayVector,    // in : Ray Unitized direction vector
   double           & rdHitT,         // out: ray parameter of 1st hit
   SmBoolean          bHitAnyThing,   // in : TRUE =     
   SmPolyFace      *& rpHitFace,      // out: 
   SmMarkType         eMarkType)      // in : uses without incrementing eMarkType on ((pPolyFace*)pNode->GetObjectList()->m_pObject)
{
  SmObjectList *sData[128];
  SmTArray < SmObjectList*> sObjsInNode(128, sData);
  SER(pNode->GetObjectList(sObjsInNode));
  
  // for every object in pNode
  ULONG ii;
  for (ii = 0; ii < sObjsInNode.GetSize(); ii++) 
    {
      SmObjectList * pObjList  = sObjsInNode[ii];
      SmPolyFace   * pPolyFace = SM_CAST_PTR(SmPolyFace, pObjList->m_pObject);
      NER(pPolyFace);
      
      // skip already processed PolyFaces
      if (pPolyFace->IsMarked(eMarkType)) 
        { continue; }  // This face has been processed before
      // mark this polyFace
      pPolyFace->Mark(eMarkType); // tag it
      
      // Do a quick Bbox test
#ifdef SM_DEBUG_CODE
      SmBoolean bDebugMe = FALSE;
      if (bDebugMe) 
        {
          smgfx_SetColor(0, 1, 0);
          pPolyFace->Draw();
          sm_GraphicsLoop();
        }
#endif
      // get PolyFace bounding sphere/Ray Distance
      SmPoint3d sSphereCenter;
      double dSphereRadius;
      SER(pPolyFace->ComputeSphereBound(FALSE, sSphereCenter, dSphereRadius));
      SmVector3d sVecToCenter = sSphereCenter - crRayPoint;
      double     dCenterT     = sVecToCenter.Dot(crRayVector);
      SmPoint3d  sPointOnRay  = crRayPoint + dCenterT * crRayVector;
      SmVector3d dVecTo       = sPointOnRay - sSphereCenter;
      double     dDistSq      = dVecTo.LengthSquared();
      double     dSphRadSq    = dSphereRadius*dSphereRadius;
      
      // skip PolyFaces whose bounding Sphere does not intersect the ray
      if (dDistSq > dSphRadSq) 
        { continue; }
      //
      SmVector3d   sNormal   = pPolyFace->GetNormal();
      SmPolyLoop * pPolyLoop = pPolyFace->GetOuterPolyLoop();
      SmPoint3d    sOrigin   = pPolyLoop->GetFirstPolyEdge()->GetStartPoint();
      double       dTParam;
      if (smgu_LinePlaneIntersect(crRayPoint, crRayVector,
                                  sOrigin, sNormal, dTParam) != SM_SUCCESS) 
        { continue; }
      SmPoint3d sPoint = crRayPoint + dTParam * crRayVector;
      if (sPoint.DistanceBetweenSquared(sSphereCenter) > dSphRadSq) 
        { continue; } // Intersection is outside of sphere radius.
      SmBoolean bInside = FALSE;
      SmBoolean b3DTest = TRUE;
      SER(pPolyFace->PointInPolygon(sPoint, bInside, b3DTest));
      if (bInside 
          && dTParam < rdHitT && dTParam > -SM_EFF_ZERO) 
        {
          rdHitT    = dTParam;
          rpHitFace = pPolyFace;
          if (bHitAnyThing) 
          { return SM_SUCCESS; }
        }
    } // end iter every pNode Object
  
  SmTreeNode * pParentNode = pNode->m_pParent;
  if (pParentNode) 
    {
      // recurse on the parent node
      SER(sm_FindPolyHit(pParentNode, 
                         crRayPoint, 
                         crRayVector,
                         rdHitT, 
                         bHitAnyThing, 
                         rpHitFace,
                         eMarkType)); // note: uses without incrementing eMarkType on ((pPolyFace*)pNode->GetObjectList()->m_pObject)
    }
  
  return SM_SUCCESS;
} // end sm_FindPolyHit


/*******************************************************************//**
PURPOSE: SHORTRAYTRACER function version.
            Fire a ray into the grid and see what we hit.

NOTES: 
***********************************************************************/
SmStatus SmRayTracer::FireRay
  (const SmPoint3d  & crRayPoint,               // in : Ray origin
   const SmVector3d & crRayVector,              // in : Ray direction, assumed:[unitized]
   double             dMaxRayParameter,         // in : Don't fire ray past this parameter.
                                                //      For an infinite ray, set
                                                //      this value to SM_BIG_DOUBLE.
   SmBoolean        & rbRayHitsSomething,       // out: TRUE = Hit something
                                                //      FALSE= Missed
   SmSolution       & rSolution,                // out:  Non-standard use of rSolution fields so use the fields 
                                                //          as described below, don't expect SmSolution::Draw to work properly.
                                                //       rSolution.m_vStart[0] = T parameter of line                  
                                                //       rSolution.m_vStart[1] = U of UVPoint          
                                                //       rSolution.m_vStart[2] = V of UVPoint                              
                                                //       rSolution.m_vStart[3] = X of 3D point  
                                                //       rSolution.m_vStart[4] = Y of 3D point                              
                                                //       rSolution.m_vStart[5] = Z of 3D point                              
                                                //       rSolution.m_vStart[6] = n.X of 3D Normal Vector       
                                                //       rSolution.m_vStart[7] = n.Y of 3D Normal Vector                              
                                                //       rSolution.m_vStart[8] = n.Z of 3D Normal Vector                              
                                                //       rSolution.m_apNodes[0]   = NULL 
                                                //       rSolution.m_apObjects[0] = Surface, edge, or vertex pointer
                                                //       rSolution.m_apObjects[1] = associated Topology pointer
                                                //                                  could be SmFace, SmEdge, or SmVertex, or NULL
   SmTArray < SmGridElement*> & rGridElements,  // out: List of elements that have an ElementBBox/Ray intersection, Ordered by ray parameter                             
   SmTArray < double>         & rMinDistances,  // out: associated ElementBBox/Ray intersection entry parameter value                             
   SmTArray < double>         & rMaxDistances)  // out: associated ElementBBox/Ray intersection exit parameter value                            
{

  // init output
  rbRayHitsSomething = FALSE;
  
#ifdef SM_DEBUG_CODE
  // draw input: ray (Start=Red, Dir=Green, End=Yellow), m_sSurfaces[i](cyan),  m_sPolyBreps[i](cyan)
SmBoolean bDebugMe = FALSE;
  if(bDebugMe) 
    {
      SmVector3d sDir    = crRayVector * dMaxRayParameter;
      SmPoint3d  sPntEnd = crRayPoint  + crRayVector * dMaxRayParameter;
      
      smgfx_Erase();
      smgfx_SetLook(1,4, 1, 0, 0); crRayPoint.Draw(); sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0, 1, 0); sDir.Draw(&crRayPoint); sm_GraphicsLoop() ;
      smgfx_SetLook(1,4, 1, 1, 0); sPntEnd.Draw(); sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 1, 1, 0) ; m_pGrid->Draw(TRUE,FALSE,FALSE,TRUE) ; sm_GraphicsLoop() ;

      for(ULONG jj=0; jj<m_sPolyBreps.GetSize(); jj++)
        { smgfx_SetLook(1,4, 0,1,1) ; m_sPolyBreps[jj]->Draw() ; sm_GraphicsLoop() ; }

      sm_GraphicsLoop();
    }
#endif  // SM_DEBUG_CODE

  // array of NewMarks to be used one for each unique context 
  // only works if SM_MT_MARK can be set for all unique contexts
  SmMarkType eMarkType = SM_MT_ALLMARKS ; 
  SM_PTR_ARRAY(sMarkLocks, SmNewMarkAndLock, 16) ;
  SmObjsDelete<SmNewMarkAndLock*> sCleanMarks(&sMarkLocks) ; 
  // SM_OBJ_ARRAY(sMarkLocks,SmNewMarkAndLock, 128) ; // gwc: When sMarkLocks->m_pData = &LocalArray everything works as designed.
  //                                                  //      When LocalArray comes into scope - the constructor is called on every member.
  //                                                  //      When LocalArray goes out of scope - the destructor is called on every member.
  //                                                  //      However, when sMarkLocks->m_pData = smos_Calloc(1, nNewMaxSize * sizeof(TYPE));
  //                                                  //        which happens when m_sPolyBreps.GetSize() > 128, the dstructor is not called on members.
  //                                                  //      This is a flaw in the SmTArray design when used for Objects rather than pointers.
  //                                                  //      I don't see a manageable way to change this - so I changed the code to 
  //                                                  //      use an SM_PTR_ARRAY with SmObjsDelete management to unlock the marks
  //                                                  // Action item - check all SM_OBJ_ARRAY uses for
  //                                                  //         1. too big an initial size - every object gets a Constructor/destructor
  //                                                  //            call when used or not.
  //                                                  //         2. Objs which depend on the destructors to do the right thing.
  //                                                  //            If the Array size exceeds its initial size a new m_pData
  //                                                  //            array is allocated - the objects in that array will not
  //                                                  //            have their destructors called when the Array goes out of scope.
  //                                                  //         3. Maybe change the SM_OBJ_ARRAY macro to make sure that destructors
  //                                                  //            are called when the Array goes out of scope.
  sMarkLocks.SetSize(m_sPolyBreps.GetSize()) ;
    
  // Use pPolyBrep->Context->Mark as a tag for new ray-firing
  if (m_sPolyBreps.GetSize() > 0) 
    {
      // for every PolyBrep - increment the mark value in the polyBrep->Context
      for(ULONG ii=0;ii<m_sPolyBreps.GetSize();ii++) 
        {
          SmPolyBrep      * pPolyBrep     = m_sPolyBreps[ii];
          const SmContext * cpContext     = pPolyBrep->GetContext();

          // make and save an unused Mark for this PolyBrep
          SmNewMarkAndLock * pThisMark = new SmNewMarkAndLock() ; 
          sMarkLocks.Add(pThisMark) ; 

          // skip PolyBrep's that share a context that has already been NewMarked 
          for(ULONG jj=0;jj<ii;jj++)
            {
              SmPolyBrep * pPolyBrep2 = m_sPolyBreps[jj];
              if (cpContext == pPolyBrep2->GetContext()) 
                {
                  continue ;
                }
            }

          // lock and increment the pPolyBrep->Context's eMarkType mark value
          SM_ASSERT_MSG(cpContext->IsMarkLocked(eMarkType) == FALSE,
                        _T("SmRayTracer::FireRay found a locked m_sPolyBreps[ii]->GetContext() SM_MT_MARK value - expects mark to be unlocked")) ;
          pThisMark->SetContext((SmContext*)cpContext, eMarkType) ; 
          pThisMark->NewMark();

          // save the available mark for use in all other contexts
          if(eMarkType == SM_MT_ALLMARKS) { eMarkType = pThisMark->GetMarkType() ; }

        } // end iter every PolyBrep incrementing Mark Values
    }

  
  // Init Grid scratch ray walking data with first Voxel/Ray intersection
  SmBoolean bHitsNothing = TRUE;
  SER(m_pGrid->Setup3DDDA(crRayPoint, crRayVector, bHitsNothing));  // note: increments m_pGrid->m_lCurrentMark

  // Quit when ray hits no voxels
  if (bHitsNothing) 
    {
      return SM_SUCCESS;
    }
  double dHitT = dMaxRayParameter;
  
#ifdef SM_DEBUG_CODE
  // draw Grid (Voxels, Stepping data, and Surfaces) (magenta), Ray (Start=red, dir=green, end=yellow)
  if(bDebugMe) 
    {
      SmVector3d sDir    = crRayVector * dMaxRayParameter;
      SmPoint3d  sPntEnd = crRayPoint  + crRayVector * dMaxRayParameter;
      
      smgfx_Erase();

      for(ULONG jj=0; jj<m_sPolyBreps.GetSize(); jj++)
        { smgfx_SetLook(1,4, 0,1,1) ; m_sPolyBreps[jj]->Draw() ; sm_GraphicsLoop() ; }

      smgfx_SetLook(1,2, 1, 1, 0); m_pGrid->Draw(TRUE,TRUE,FALSE,TRUE) ; sm_GraphicsLoop() ;   // draw all surfaces
      smgfx_SetLook(1,2, 1, 1, 0); m_pGrid->Draw(TRUE,TRUE,TRUE,FALSE) ; sm_GraphicsLoop() ;   // draw voxel surfaces
      smgfx_SetLook(1,4, 1, 0, 0); crRayPoint.Draw(); sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0, 1, 0); sDir.Draw(&crRayPoint); sm_GraphicsLoop() ;
      smgfx_SetLook(1,4, 1, 1, 0); sPntEnd.Draw(); sm_GraphicsLoop() ;
      sm_GraphicsLoop();
    }
#endif  // SM_DEBUG_CODE

  
  // traverse ray from voxel to voxel looking for 1st GridElement Intersection.
  // Stop when ray intersects an element or when ray exits grid.
  while (TRUE) 
    {
      // on each loop - move to next voxel that contains
      // any GridElementBBox/Ray intersections.  
      //   For every GridElementBBox/Ray intersection
      //     - look for ray/ElementGeometry intersection
      //     - save first intersection.
      //   If(Found Ray/GridElement Intersection) break ;
      //   else loop again.
      SmBoolean bExitsGrid;
      long lVoxelAdd[3];

      // traverse ray moving from voxel to voxel until a voxel is found that 
      // contains elements, not yet marked, whose element BBoxes intersect the ray. Then return 
      // list of all GridElementBBox/Ray intersections in that voxel and set walking to next voxel.
      //  bExitsGrid = FALSE when an element was found
      //               TRUE  when ray exited grid before finding an element
      SER(m_pGrid->Step3DDDA(SM_BIG_DOUBLE, 
                             bExitsGrid, 
                             lVoxelAdd,
                             rGridElements,     // out: elements whose BBoxes intersect the ray, sorted by increasing dTEnter values
                             rMinDistances,     // out: associated ElementBBox/Ray intersection dTEnter values                      
                             rMaxDistances));   // out: associated ElementBBox/Ray intersection dTExit values                       
                                                // note: uses without incrementing SmGrid::m_lCurrentMark value
#ifdef SM_DEBUG_CODE
      if (bDebugMe) 
        {
          SmVector3d sDir    = crRayVector * dMaxRayParameter;
          SmPoint3d  sPntEnd = crRayPoint  + crRayVector * dMaxRayParameter;
      
          smgfx_Erase();
          smgfx_SetLook(1,2, 1, 1, 0); m_pGrid->Draw(TRUE,TRUE,TRUE,FALSE) ; sm_GraphicsLoop() ; // draw grid without voxels to show stepping data
          smgfx_SetLook(1,4, 1, 0, 0); crRayPoint.Draw(); sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 0, 1, 0); sDir.Draw(&crRayPoint); sm_GraphicsLoop() ;
          smgfx_SetLook(1,4, 1, 1, 0); sPntEnd.Draw(); sm_GraphicsLoop() ;
          sm_GraphicsLoop();

          // draw ray segments that lie within a grid object's bounding box
          if(bExitsGrid == FALSE)
            {
              for(ULONG di=0;di<rGridElements.GetSize();di++)
                {
                  //SmGridElement * pGridElement = rGridElements[di] ;
                  double dParam1 = rMinDistances[di] ;
                  double dParam2 = rMaxDistances[di] ;
                  SmPoint3d  sSegStart  = crRayPoint + crRayVector * dParam1 ;
                  SmVector3d sSegVector = crRayPoint + crRayVector * dParam2 - sSegStart ;
          
                  smgfx_SetLook(2+2*di,4+2*di, 1,0,0) ; sSegVector.Draw(&sSegStart) ; sm_GraphicsLoop() ;
                  sm_GraphicsLoop();
                }
            }
          sm_GraphicsLoop();
        }
#endif  // SM_DEBUG_CODE

      // For every GridElement found in the current Voxel
      for (ULONG ii = 0; ii < rGridElements.GetSize(); ii++)
        {
          // skip hits which come after current hit pramameter value
          if (rMinDistances[ii] > dHitT) 
            { continue; }

          // skip hits on the current SkipNode
          if (m_pSkipElement && m_pSkipElement == rGridElements[ii])
            { continue; }

          // init exit flag
          SmBoolean bFoundIntersection = FALSE;

          // handle PolyBrep Faces
          if (rGridElements[ii]->GetType() == SmPolygonGridElement_TYPE) 
            {
              // get SmGridElement's SmTreeNode - PolyBreps store SmTreeNodes in SmGridElements
              SmTreeNode *pNode = (SmTreeNode *)((SmPolygonGridElement *)rGridElements[ii])->GetElement();

              // Handle polygon ray firing
              SmPolyFace * pHitFace = NULL;
              SER(sm_FindPolyHit(pNode, 
                                 crRayPoint, 
                                 crRayVector,
                                 dHitT, 
                                 m_bHitAnyThing, 
                                 pHitFace,
                                 eMarkType));  // in : uses without incrementing eMarkType on ((pPolyFace*)pNode->GetObjectList()->m_pObject)
              if (pHitFace) 
                {
                  SmPoint3d sHitPnt         = crRayPoint + crRayVector*dHitT;
                  SmVector3d sHitNormal     = pHitFace->GetNormal() ;
                  rSolution.m_eSolutionType = SM_ST_SINGLE_VALUE ;
                  rSolution.m_lNumVariables = 1 ;
                  rSolution.m_vStart[0]     = dHitT;          // ray parameter
                  rSolution.m_vStart[1]     = SM_BIG_DOUBLE;
                  rSolution.m_vStart[2]     = SM_BIG_DOUBLE;
                  rSolution.m_vStart[3]     = sHitPnt.x;      // ray/face intersection point
                  rSolution.m_vStart[4]     = sHitPnt.y;
                  rSolution.m_vStart[5]     = sHitPnt.z;
                  rSolution.m_vStart[6]     = sHitNormal.x ;  // face normal at xsect
                  rSolution.m_vStart[7]     = sHitNormal.y ;
                  rSolution.m_vStart[8]     = sHitNormal.z ;
                  rSolution.m_lNumObjects   = 1;
                  rSolution.m_apObjects[0]  = pHitFace;
                  rSolution.m_apObjects[1]  = pHitFace;
                  rbRayHitsSomething        = TRUE;
                  if (m_bHitAnyThing) 
                    { break; }
                }
              continue;
            }


          // arrive here when GridElement is not a PolygonGrid Element

          // handle Brep Surfaces          

          // init solver parameters
          SmBoolean bNeedsSubdivision;
          m_sLSISolver.m_vLinePoint  = crRayPoint;
          m_sLSISolver.m_vLineVector = crRayVector;
          m_sLSISolver.m_dTMinOfBBox = rMinDistances[ii];
          m_sLSISolver.m_dTMaxOfBBox = rMaxDistances[ii];

          // skip ElementBBox/Ray intersections that happen before the ray Start Point
          if (m_sLSISolver.m_dTMaxOfBBox < -SM_EFF_ZERO) 
            { continue; }

          // solve for a ray/GridElementGeometry intersection
          SmSolution sSolution;
          SM_ASSERT(rGridElements[ii]->GetType() == SmSurfaceGridElement_TYPE) ;
          m_sLSISolver.LocalSolve((SmSurfaceGridElement *)rGridElements[ii],
                                  bNeedsSubdivision,
                                  bFoundIntersection, 
                                  sSolution);
#ifdef SM_DEBUG_CODE
          if(bDebugMe)
            {
              sSolution.Dump() ;
              smgfx_Erase() ;
              smgfx_SetLook(1,2, 1,1,0) ; m_pGrid->Draw(TRUE,TRUE,TRUE,FALSE) ; sm_GraphicsLoop() ;
              smgfx_SetLook(2,3, 0,1,0) ; ((SmSurfaceGridElement *)rGridElements[ii])->m_pSurface->DrawUV() ; sm_GraphicsLoop() ;
              smgfx_SetLook(4,6, 1,0,0) ; if(bFoundIntersection) sSolution.Draw(1) ; sm_GraphicsLoop() ;
              sm_GraphicsLoop() ;
            }
#endif // SM_DEBUG_CODE
          // when an intersection was found 
          // that happens before any others and 
          // is in the positive part of the ray
          if (   bFoundIntersection 
              && sSolution.m_vStart[0] < dHitT 
              && sSolution.m_vStart[0] > -SM_EFF_ZERO)
          {
              // put the solution in the output
              dHitT = sSolution.m_vStart[0];
              rSolution.m_vStart.m_dSolutionValue = sSolution.m_vStart.m_dSolutionValue;
              rSolution.m_eSolutionType           = sSolution.m_eSolutionType ;
              
              rSolution.m_vStart[0] = sSolution.m_vStart[0]; // Put T parameter of line in 0
              rSolution.m_vStart[1] = sSolution.m_vStart[1]; // Put UV Point in 1-2
              rSolution.m_vStart[2] = sSolution.m_vStart[2];
              rSolution.m_vStart[3] = sSolution.m_vStart[3]; // Put 3D point in 3-5
              rSolution.m_vStart[4] = sSolution.m_vStart[4];
              rSolution.m_vStart[5] = sSolution.m_vStart[5];
              rSolution.m_vStart[6] = sSolution.m_vStart[6]; // Put 3D Normal into 6-8
              rSolution.m_vStart[7] = sSolution.m_vStart[7];
              rSolution.m_vStart[8] = sSolution.m_vStart[8];
              
              rSolution.m_apNodes[0]   = NULL ; // GWC no longer set as output
                                                //   since SurfaceCaches are now temporary
                                                //   structures.
                                                // was: sSolution.m_apNodes[0];
              rSolution.m_lNumObjects  = 2;
              rSolution.m_apObjects[0] = sSolution.m_apObjects[0]; // surface pointer
              rSolution.m_apObjects[1] = sSolution.m_apObjects[1]; // face pointer or NULL
              
#ifdef SM_DEBUG_CODE
              if (bDebugMe) 
              {
                  SmPoint3d sPnt(sSolution.m_vStart[3], sSolution.m_vStart[4], sSolution.m_vStart[5]);
                  SmSurface *pS = SM_CAST_PTR(SmSurface, sSolution.m_apObjects[0]);
                  if (pS) 
                  {
                      smgfx_Erase();
                      smgfx_SetColor(1, 0, 0);
                      sPnt.Draw();
                      sm_GraphicsLoop();
                      smgfx_SetColor(0, 0, 1);
                      SmFace *pF =(SmFace*)pS->GetFace();
                      if (pF) 
                      {
                          pF->Draw();
                          sm_GraphicsLoop();
                          pF->GetBrep()->Draw();
                          sm_GraphicsLoop();
                      }
                  }
              }
#endif // SM_DEBUG_CODE
              
              rbRayHitsSomething = TRUE;
              // If we are looking for any hit just stop here otherwise
              // make sure we get the closest hit.
              if (m_bHitAnyThing)
                  break;
            } // end found a keeper intersection check


        } // end iter every found Grid Element
      
      // Stop when ray exits grid
      if (bExitsGrid) 
        { break; }
      
      // Stop when ray hit an element 
      if (dHitT < m_pGrid->m_dRayT) 
        { break; }

    } // end while(TRUE) stepping from voxel to voxel
  
  // all done
  return SM_SUCCESS;

} // end SmRayTracer::FireRay

/*******************************************************************//**
PURPOSE: Add this surface to the grid used by the raytracer as
            an SmHierarchyElement added to the Grid's Hierarchy element list.

NOTES: effects:
  1. Build SurfaceCache for Surface.
  2. Add Surface and Cache to RayTracer's surface and Cache lists.
  3. Add Surface and Cache to Grid as a hierarchical element.
***********************************************************************/
SmStatus SmRayTracer::AddSurfaceToGrid(SmSurface *pSurface)          
{
  // locals
  //SmObject       *pOwner          = pSurface->GetOwner();
  //SmFace         *pFace           = SM_CAST_PTR(SmFace, pOwner);
  //double          dAngleTolerance = dAngTolDeg * SM_PI / 180.0;
  SmExtent3d      sSurfBBox;
  
  // build the surface spatial decomposition by building the surface cache
  // when surface is a face
  // build surface cache and place it in the global Cache Queue
  SmSurfaceCache *pSurfaceCache = smsurf_GetSurfaceCache(pSurface) ;
  if(pSurfaceCache == NULL || pSurfaceCache->GetTree() == NULL )
      return SM_ERR;
  SmTreeNode *pNode = pSurfaceCache->GetTree()->GetTopNode();

#ifdef SM_DEBUG_CODE
  SmBoolean bDebugMe = FALSE;
  if (bDebugMe) 
    {
      smgfx_Erase();
      smgfx_SetLook(1,2, 0,1,1) ; pSurface->DrawUV(3, 3); sm_GraphicsLoop();
      smgfx_SetLook(1,2, 0,0,1) ; if(m_pGrid) m_pGrid->Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,0) ; if(pSurfaceCache) pSurfaceCache->Draw(TRUE,FALSE,FALSE,FALSE,TRUE) ; sm_GraphicsLoop() ;
      sm_GraphicsLoop();
    }
#endif

  // set the bounding box = cached normal box increased by a small amount
  sSurfBBox = pNode->m_sBBox;

  // Expand it a little just to be safe
  SmPoint3d sCent;
  double dRadius = 0.0;
  sSurfBBox.ComputeSphereBound(sCent,dRadius);
  sSurfBBox.ExpandAbsolute(dRadius/20.0);

  // store the surface in the RayTracer lists
  // gwc: no longer store the SurfaceCache in the RayTracer lists
  m_sSurfaces.Add(pSurface);

  // add the surface to the grid's hierarchical element list
  SmSurfaceHierarchyElement * pSH = new(m_crContext) SmSurfaceHierarchyElement(pSurface, m_dRayTolerance);
  m_pGrid->AddHierarchyElement(pSH, sSurfBBox);

  // Expand the hierarchy element to avoid recomputing the surface cache a 2nd time
  // The SurfaceCache is passed through the global cache queue.
  // gwc option: if the global cache queue is just in the way then
  //      1. replace smsurf_GetSurfaceCache(pSurface) with
  //         SmCacheMgr::CacheMakeOrValidate()
  //      2. Pass that cache to ExpandHierarchy() as an argument.
  //      3. Delete the ObjectCache after ExpandHierarchy() has run.    
  pSH->ExpandHierarchy(m_pGrid) ;

#ifdef SM_DEBUG_CODE
  if (bDebugMe) 
    {
      smgfx_Erase();
      smgfx_SetLook(1,2, 0,1,1) ; pSurface->DrawUV(3, 3); sm_GraphicsLoop();
      smgfx_SetLook(1,2, 0,0,1) ; if(m_pGrid) m_pGrid->Draw(TRUE,TRUE,TRUE) ; sm_GraphicsLoop() ;
      sm_GraphicsLoop();
    }
#endif
  
  // all done
  return SM_SUCCESS;

} // end SmRayTracer::AddSurfaceToGrid



/*******************************************************************//**
PURPOSE: Add this polybrep to the grid used by the raytracer.

NOTES: 
***********************************************************************/
SmStatus SmRayTracer::AddPolyBrepToGrid
  (SmPolyBrep * pPolyBrep,             // in : 
   double       dExpansionTolerance)   // in : 
{
  // warn PolyBrep its about to be modifided
  pPolyBrep->Notify(SM_NO_PRE_EDIT, pPolyBrep, NULL, NULL); 
  
  // Compute a Brep Cache - a spatial decomposition of 
  //  the vertices, edges and faces contained in 3 SmTree objects used
  //  as a VertexTree, an EdgeTree, and a FaceTree 
  SmBrepCache *pBC =(SmBrepCache*)SmCacheMgr::GetOrCreateObjectCache(SM_OC_BREP, pPolyBrep); 
  NER(pBC);

  // get all Surface Tree Nodes
  SmTree * pTree = pBC->GetSurfaceTree();
  NER(pTree);
  SmTArray < SmTreeNode*> sAllTreeNodes;
  pTree->GetAllTreeNodes(sAllTreeNodes);

  // For every Surface Tree Noe
  ULONG ii;
  for (ii = 0; ii < sAllTreeNodes.GetSize(); ii++) 
    {
      SmTreeNode *pNode = sAllTreeNodes[ii];

      // skip non-leaf nodes
      if (pNode->m_pChild1 != NULL) 
        { continue; }

      // Get PNode's boundingBox increased by Tol
      SmExtent3d sBBox = pNode->m_sBBox;
      sBBox.ExpandAbsolute(dExpansionTolerance);

      // Add leaf node to Grid
      //  -   make an SmGridElement object containing a pointer to pNode and BoundingBox data. 
      //      Then for every voxel that intersects the pNode's bounding box,
      //      add a pointer to the SmGridElement to the voxel's m_pElements list.
      m_pGrid->AddElement(pNode, 
                          SM_GE_POLYBREP,
                          sBBox);

    } // end iter every Surface Tree Node

  // Add the PolyBrep to the SmRayTracer PolyBrep List
  m_sPolyBreps.Add(pPolyBrep);


  // all done
  return SM_SUCCESS;

} // end SmRayTracer::AddPolyBrepToGrid

/*******************************************************************//**
PURPOSE: Compute the total size of the memory used by the SmRayTracer.

NOTES:
***********************************************************************/
ULONG SmRayTracer::GetMemoryUsed      // rtn: Total Memory being used for all objects
  (ULONG &rlMemoryAllocated,          // out: Total Memory allocated for all objects
   SmBoolean bAddSurfaceMemory,       // in : TRUE = Add in Surface Object Memory
                                      //      FALSE= Don't, Surfaces are owned by other Object
                                      //                     don't need to be counted twice.
                                      //      default:[TRUE]
   ULONG *pOptGlobalCacheAllocated,   // NotUsed: out: Optional Global Cache  Allocated Memory size, NULL to ignore, default:[NULL]
   ULONG *pOptGridAllocated,          // out: Optional Grid          Allocated Memory size, NULL to ignore, default:[NULL]
   ULONG *pOptSurfaceArrayAllocated,  // out: Optional Surface Array Allocated Memory size, NULL to ignore, default:[NULL]
   ULONG *pOptSurfaceCacheAllocated,  // out: Optional Surface Cache Allocated Memory size, NULL to ignore, default:[NULL]
   ULONG *pOptGlobalCacheUsed,        // NotUsed: out: Optional Global Cache  Used      Memory size, NULL to ignore, default:[NULL]
   ULONG *pOptGridUsed,               // out: Optional Grid          Used      Memory size, NULL to ignore, default:[NULL]
   ULONG *pOptSurfaceArrayUsed,       // out: Optional Surface Array Used      Memory size, NULL to ignore, default:[NULL]
   ULONG *pOptSurfaceCacheUsed)       // out: Optional Surface Cache Used      Memory size, NULL to ignore, default:[NULL]
 const
{
  SM_REF2(pOptGlobalCacheAllocated, pOptGlobalCacheUsed) ; 
  // locals
  ULONG ii, lThisUsed ;

#ifdef SM_USE_GLOBAL_CACHE
  // Context Global Cache Memory
  SmGlobalCacheSize sGlobalCacheSize ;                                                      
  m_crContext.GetGlobalCacheSize(sGlobalCacheSize) ;

  ULONG lGlobalCacheUsed      = sGlobalCacheSize.GetMemoryUsed() ;
  ULONG lGlobalCacheAllocated = sGlobalCacheSize.GetMemoryAllocated() ;
#endif

  // SmGrid Memory
  ULONG lGridAllocated = 0 ;
  ULONG lGridUsed      = m_pGrid ? m_pGrid->GetMemoryUsed(lGridAllocated) : 0 ;

  // Surface Array and SurfaceCache Array Memory
  ULONG lSurfaceCacheUsed      = 0, lSurfaceArrayUsed      = 0 ;
  ULONG lSurfaceCacheAllocated = 0, lSurfaceArrayAllocated = 0 ;

  // when asked - get Surface Array Memory
  if(bAddSurfaceMemory)
    {
      // Surface Array Memory
      lSurfaceArrayUsed = m_sSurfaces.GetMemoryUsed(lSurfaceArrayAllocated) ;
  
      // Surface Object memory 
      for(ii=0;ii<m_sSurfaces.GetSize();ii++)
        {
          // SmSurface *pSurface = m_sSurfaces[ii] ;

          // TODO:  Need to add memory tools to core SmObjects
          //      // get memory for this Surface
          //      lThisUsed = pSurface->GetMemoryUsed(lThisAllocated) ;
          //      
          //      // accumulate
          //      lSurfaceCacheUsed      += lThisUsed ;
          //      lSurfaceCacheAllocated += lThisAllocated ;
   
        } // end iter every SurfaceCache
    } // end if adding Surface Memory check
   
 
  // PolyBrep Array Memory
  ULONG lPolyBrepUsed      = 0 ;
  ULONG lPolyBrepAllocated = 0 ;

      // PolyBrep Array Memory
      lPolyBrepUsed = m_sPolyBreps.GetMemoryUsed(lPolyBrepAllocated) ;
  
      // PolyBrep Object memory 
      for(ii=0;ii<m_sPolyBreps.GetSize();ii++)
        {
          // SmPolyBrep *pPolyBrep = m_sPolyBreps[ii] ;

          // TODO:  Need to add memory tools to core SmObjects
          //      // get memory for this PolyBrep
          //      lThisUsed = pPolyBrep->GetMemoryUsed(lThisAllocated) ;
          //      
          //      // accumulate
          //      lPolyBrepUsed      += lThisUsed ;
          //      lPolyBrepAllocated += lThisAllocated ;
   
        } // end iter every PolyBrepCache

  // all done
  rlMemoryAllocated =   sizeof(*this)
                      + sizeof(m_crContext) 
#ifdef SM_USE_GLOBAL_CACHE
                      + lGlobalCacheAllocated
#endif // SM_USE_GLOBAL_CACHE
                      + lGridAllocated
                      + lSurfaceCacheAllocated
                      + lSurfaceArrayAllocated
                      + lPolyBrepAllocated ;

  lThisUsed         =   sizeof(*this)
                      + sizeof(m_crContext) 
#ifdef SM_USE_GLOBAL_CACHE
                      + lGlobalCacheUsed
#endif // SM_USE_GLOBAL_CACHE
                      + lGridUsed
                      + lSurfaceCacheUsed
                      + lSurfaceArrayUsed
                      + lPolyBrepUsed ;

  // set outputs
  if(pOptGridAllocated)         *pOptGridAllocated         = lGridAllocated ;        
  if(pOptSurfaceArrayAllocated) *pOptSurfaceArrayAllocated = lSurfaceArrayAllocated ;
  if(pOptSurfaceCacheAllocated) *pOptSurfaceCacheAllocated = lSurfaceCacheAllocated ;
  if(pOptGridUsed)              *pOptGridUsed              = lGridUsed ;             
  if(pOptSurfaceArrayUsed)      *pOptSurfaceArrayUsed      = lSurfaceArrayUsed ;     
  if(pOptSurfaceCacheUsed)      *pOptSurfaceCacheUsed      = lSurfaceCacheUsed ;     
#ifdef SM_USE_GLOBAL_CACHE
  if(pOptGlobalCacheAllocated)  *pOptGlobalCacheAllocated  = lGlobalCacheAllocated ; 
  if(pOptGlobalCacheUsed)       *pOptGlobalCacheUsed       = lGlobalCacheUsed ;      
#endif // SM_USE_GLOBAL_CACHE

  // all done
  return(lThisUsed) ;

} // end SmRayTracer::GetMemoryUsed

/*******************************************************************//**
PURPOSE: Debug Formatted print of SmRayTracer

NOTES: 
***********************************************************************/
void SmRayTracer::Dump()   
  const
{
  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE] ;

  // type, m_iElemCount, and m_iTotalBasisCount
  smos_sprintf(sBuff       ,_T("\n  SmRayTracer[0x%p]: Context[0x%p], RayTolerance[%16.16lf], RayAccuracy[%16.16lf]"), 
             this, &m_crContext, m_dRayTolerance, m_dRayAccuracy) ; 
  smos_sprintf(sBuffForFile,_T("\n  SmRayTracer[%s]: Context[%s], RayTolerance[%16.16lf], RayAccuracy[%16.16lf]"), 
             _T("NotNULL"), 
             _T("NotNULL"),  
             m_dRayTolerance,  
             m_dRayAccuracy) ; 
  smos_WriteBuffer(sBuff, sBuffForFile);

  // Grid Size
  smos_sprintf(sBuff,_T("\n         Grid[0x%p]: Size[%ld x %ld x %ld]"),
             m_pGrid,
             m_pGrid ? m_pGrid->m_lSize[0] : 0, 
             m_pGrid ? m_pGrid->m_lSize[1] : 0, 
             m_pGrid ? m_pGrid->m_lSize[2] : 0) ;
  smos_sprintf(sBuffForFile,_T("\n         Grid[%s]: Size[%ld x %ld x %ld]"),
             m_pGrid ? _T("NotNULL") : _T("NULL"),
             m_pGrid ? m_pGrid->m_lSize[0] : 0, 
             m_pGrid ? m_pGrid->m_lSize[1] : 0, 
             m_pGrid ? m_pGrid->m_lSize[2] : 0) ;
  smos_WriteBuffer(sBuff, sBuffForFile);


  // Surface Array and Surface Cache Array sizes

  // PolyBrep Size
  smos_sprintf(sBuff,_T(", PolyBrep Cnt[%ld]"),  m_sPolyBreps.GetSize()) ; 
  smos_WriteBuffer(sBuff);

  // Memory Usage Summary
  ULONG lAllocated ;
  ULONG lGlobalCacheAllocated = 0 ; 
  ULONG lGridAllocated ;        
  ULONG lSurfaceArrayAllocated ;
  ULONG lSurfaceCacheAllocated ;
  ULONG lGlobalCacheUsed = 0 ;      
  ULONG lGridUsed ;             
  ULONG lSurfaceArrayUsed ;     
  ULONG lSurfaceCacheUsed ;     

  ULONG lUsed = GetMemoryUsed(lAllocated, TRUE,
                              &lGlobalCacheAllocated,
                              &lGridAllocated,
                              &lSurfaceArrayAllocated,
                              &lSurfaceCacheAllocated,
                              &lGlobalCacheUsed,
                              &lGridUsed,
                              &lSurfaceArrayUsed,
                              &lSurfaceCacheUsed) ;

  smos_sprintf(sBuff,_T("\n  Allocated Bytes: Total[%ld], GlobalCache[%ld], Grid[%ld], SurfaceArray[%ld], SurfaceCache[%ld]"),
             lAllocated,
             lGlobalCacheAllocated, 
             lGridAllocated,        
             lSurfaceArrayAllocated,
             lSurfaceCacheAllocated) ;
  smos_WriteBuffer(sBuff);
  smos_sprintf(sBuff,_T("\n  Used      Bytes: Total[%ld], GlobalCache[%ld], Grid[%ld], SurfaceArray[%ld], SurfaceCache[%ld]"),
             lUsed,
             lGlobalCacheUsed, 
             lGridUsed,        
             lSurfaceArrayUsed,
             lSurfaceCacheUsed) ;
  smos_WriteBuffer(sBuff);

  smos_WriteBuffer(_T("\n\n"));

} // end SmRayTracer::Dump
