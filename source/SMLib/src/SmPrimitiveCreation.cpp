// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmPrimitiveCreation.cpp
* PURPOSE: Implementation of class members
**********************************************************************/

#include "StdAfx.h"

#include <nurbs.h>
#include <SmPrimitiveCreation.h>
#include <SmTransSweepGeometry.h>
#include <SmRotationalSweepGeometry.h>
#include <SmTopologyIntersector.h>
#include <SmCompositeCurve.h>
#include <SmPlane.h>
#include <SmSphere.h>
#include <SmCone.h>
#include <SmTorus.h>
#include <SmSurfOfExtrusion.h>
#include <SmCircle.h>
#include <SmGraphicsOutput.h>
#include <SmGeomUtility.h>
#include <SmCylinder.h>
#include <SmLine.h>
#include <SmAssertArray.h>
#include <SmMerge.h>

#ifdef SM_DEBUG_CODE
#include <SmCrvOnSurf.h>
#endif // SM_DEBUG_CODE

// Temp define to allow switching back to old algorithms
#define USE_NEW_ANALYTICS

/*******************************************************************//**
PURPOSE: Create a Brep which contains one face that is a
            rectangle primitive.  

NOTES: 
***********************************************************************/
SmBrep * SmPrimitiveCreation::CreateRectangle
 (const SmContext        & crContext,         // in : context for new object construction
  double                   dToleranceOfBrep,  // in : Brep tolerance
  double                   dWidth,            // in : Width along X Axis of the reference frame.
  double                   dHeight,           // in : Height along Y Axis of the reference frame.
  const SmAxis2Placement & crRefFrame)        // in : origin = bottom right corner of the surface,
                                              //      note: surface made in the RefFram XY plane's positive quadrant.
{
  // check inputs
  SM_ASSERT(dWidth > dToleranceOfBrep);
  SM_ASSERT(dHeight > dToleranceOfBrep);

  // rectangle corner points
  const SmPoint3d &rP00 = crRefFrame.GetOriginRef();
  const SmPoint3d  sP10 = rP00 + dWidth  * crRefFrame.GetXAxisRef();
  const SmPoint3d  sP01 = rP00 + dHeight * crRefFrame.GetYAxisRef();
  const SmPoint3d  sP11 = sP10 + sP01 - rP00;

  // BSpline surface from the corner points
  SmBSplineSurface *pBSS = NULL ;
  SE(SmBSplineSurface::CreateBilinearSurface(crContext, rP00, sP10, sP01, sP11, pBSS));
  SmObjDelete sCleanup1(pBSS);

  // Build an SmPlane from the BSpline (when analytics are being used)
  SmSurface *pPlane;

  // Copy pBSS for new face, when possible as an analytic surface
  SE(pBSS->CopyAndAddAnalytics(crContext, pPlane));
  SmObjDelete sCleanup2(pPlane);
  
  // Brep object
  SmBrep *pBrep = new (crContext) SmBrep();
  SmObjDelete sCleanBrep(pBrep);

  // no work - Brep construction failure
  if (!pBrep) { SE(SM_ERR); return NULL; }

  // gwc: removed next line - sets pBrep->Tol = crContext::ZoneTol3d
  //   pBrep->SetTolerance(dToleranceOfBrep);

  // build a face in the Brep from the SmPlane
  SmFace *pNewFace = NULL ;
  SE(pBrep->CreateFaceFromSurface(pPlane, pPlane->GetNaturalUVDomain(), pNewFace));

  // all done - clear the ObjDeletes and return
  sCleanBrep.Clear();
  sCleanup2.Clear();
  return pBrep;

} // end SmPrimitiveCreation::CreateRectangle

/*******************************************************************//**
PURPOSE: Add a bounded rectangular plane to this brep

NOTES: 
***********************************************************************/
SmStatus SmPrimitiveCreation::CreateRectangle
 (double                   dWidth,            // in : Width along X Axis of the reference frame.
  double                   dHeight,           // in : Height along Y Axis of the reference frame.
  const SmAxis2Placement & crRefFrame)        // in : origin = bottom right corner of the surface,
                                              //      note: surface made in the RefFram XY plane's positive quadrant.
{
    SmBrep* pBrep = m_pRegion->GetBrep();
    if (!pBrep) { SE(SM_ERR); return SM_ERR; }

    const SmContext* pContext = pBrep->GetContext();
    double dToleranceOfBrep = pBrep->GetTolerance();

    // check inputs
    SM_ASSERT(dWidth > dToleranceOfBrep);
    SM_ASSERT(dHeight > dToleranceOfBrep);

    // rectangle corner points
    const SmPoint3d &rP00 = crRefFrame.GetOriginRef();
    const SmPoint3d  sP10 = rP00 + dWidth  * crRefFrame.GetXAxisRef();
    const SmPoint3d  sP01 = rP00 + dHeight * crRefFrame.GetYAxisRef();
    const SmPoint3d  sP11 = sP10 + sP01 - rP00;

    // BSpline surface from the corner points
    SmBSplineSurface *pBSS = NULL ;
    SE(SmBSplineSurface::CreateBilinearSurface(*pContext, rP00, sP10, sP01, sP11, pBSS));
    SmObjDelete sCleanup1(pBSS);

    // Build an SmPlane from the BSpline (when analytics are being used)
    SmSurface *pPlane;

    // Copy pBSS for new face, when possible as an analytic surface
    SE(pBSS->CopyAndAddAnalytics(*pContext, pPlane));
    SmObjDelete sCleanup2(pPlane);

    // build a face in the Brep from the SmPlane
    SmFace *pNewFace = NULL ;
    SE(pBrep->CreateFaceFromSurface(pPlane, pPlane->GetNaturalUVDomain(), pNewFace));

    // all done - clear the ObjDeletes and return
    sCleanup1.Clear();
    sCleanup2.Clear();
    return SM_SUCCESS;

} // end SmPrimitiveCreation::CreateRectangle

/*******************************************************************//**
PURPOSE: Create a Brep which contains one face that is bounded by a circle.

NOTES: 
***********************************************************************/
SmBrep * SmPrimitiveCreation::CreateCircle
 (const SmContext  & crContext,           // in : context for new object construction
  double             dToleranceOfBrep,    // in : Brep Tolerance
  double             dRadius,             // in : Radius of circle
  const SmAxis2Placement & crRefFrame)    // in : Center and orientation of the circle.
                                          //      circle is built centered on the RefFrame's origin in the XY plane
{
  // check input
  SM_ASSERT(dRadius > dToleranceOfBrep);

  // Brep - temporarily schedule the Brep for deletion (in case of error)
  SmBrep *pBrep = new (crContext) SmBrep();
  if (!pBrep) { SE(SM_ERR); return NULL; }
  SmObjDelete sCleanup(pBrep);
  // gwc: removed next line - sets pBrep->Tol = crContext::ZoneTol3d
  //   pBrep->SetTolerance(dToleranceOfBrep);

  // BSplineCurve Circle
  SmBSplineCurve *pCircle = NULL ;
  SE(SmBSplineCurve::CreateCircleSegment(crContext,
                                         3,
                                         crRefFrame,
                                         dRadius,
                                         0.0,
                                         360.0,
                                         SM_CO_QUADRATIC,
                                         pCircle));
  if (!pCircle) { SE(SM_ERR); return NULL; }

  // Build face from 3D Curves
  SmTArray<SmCurve*> sCurves;
  sCurves.Add(pCircle);
  SmFace *pNewFace = NULL ;
  pBrep->m_bEditingEnabled = TRUE;
  if (pBrep->CreatePlanarFaceWith3DCurves(pBrep->GetInfiniteRegion(), // in : region to contain newFace - not checked
                                                                      //      NULL for infinite region.
                                          sCurves,                    // in : array of planar curves to bound newFace - curves used by NewEdges
                                          dToleranceOfBrep,           // in : 3d Distance for planarity checks
                                          pNewFace) != SM_SUCCESS)    // out: new face
    {
      SE(SM_ERR);
      return NULL;
    }

  //pNewFace->ShrinkGeometry();

  // all done - clear temporary delete objs, reset Brep bit and return
  sCleanup.Clear();
  pBrep->m_bEditingEnabled = FALSE;
  return pBrep;

} // end SmPrimitiveCreation::CreateCircle

/*******************************************************************//**
PURPOSE: Add a bounded circular planar Face to this brep

NOTES: 
***********************************************************************/
SmStatus SmPrimitiveCreation::CreateCircle
 (double                   dRadius,     // in : Radius of circle
  const SmAxis2Placement & crRefFrame)  // in : Center and orientation of the circle.
                                        //      circle is built centered on the RefFrame's origin in the XY plane
{
    SmBrep* pBrep = m_pRegion->GetBrep();
    if (!pBrep) { SE(SM_ERR); return SM_ERR; }

    const SmContext* pContext = pBrep->GetContext();
    double dToleranceOfBrep = pBrep->GetTolerance();

    // check input
    SM_ASSERT(dRadius > dToleranceOfBrep);

    pBrep->m_bEditingEnabled = TRUE;

    SmCircle* pCircle = NULL;
    SmExtent1d sDomain( 0.0, 360.0 );
    SmCircle::CreateCanonical( *pContext, crRefFrame, dRadius, pCircle, &sDomain );
    
    if (!pCircle) { SE(SM_ERR); return SM_ERR; }

    // Build face from 3D Curves
    SmTArray<SmCurve*> sCurves;
    sCurves.Add(pCircle);

    SmFace *pNewFace = NULL ;
    if (pBrep->CreatePlanarFaceWith3DCurves(pBrep->GetInfiniteRegion(), // in : region to contain newFace - not checked
                                                                        //      NULL for infinite region.
                                            sCurves,                    // in : array of planar curves to bound newFace - curves used by NewEdges
                                            dToleranceOfBrep,           // in : 3d Distance for planarity checks
                                            pNewFace) != SM_SUCCESS)    // out: new face
      {
        SE(SM_ERR);
        return SM_ERR;
      }

    //pNewFace->ShrinkGeometry();

    pBrep->m_bEditingEnabled = FALSE;
    return SM_SUCCESS;

} // end SmPrimitiveCreation::CreateCircle

/*******************************************************************//**
PURPOSE: Create a rectilinear solid in the given region of the Brep.
            
NOTES: 
  Builds rectilinear solid with a corner located at the input rRefFrame origin
  and extending by 
     dLength in the rRefFrame X direction, 
     dWidth  in the rRefFrame Y direction, and
     dHeight in the rRefFrame Z diretion.

  If the vertices/edges of the cube happen to coincide
  with any vertices/edges of the Brep, they will be 
  stitched together (cf SmStitch).

  The Brep is assumed to have a valid tolerance set.

  For now, the region must be the infinite region of the 
  Brep (limitation of SmStitch).
***********************************************************************/
SmStatus SmPrimitiveCreation::CreateBox
 (double                   dLength,   // in : extent along X axis of frame
  double                   dWidth,    // in : extent along Y axis of frame
  double                   dHeight,   // in : extent along Z axis of frame
  const SmAxis2Placement & rRefFrame) // in : defines box min corner and orientation
{
  // locals
  SmBrep           * pBrep    = m_pRegion->GetBrep();
  const SmContext  * pContext = pBrep->GetContext();
  SmPlane          * pPlane   = NULL;
  SmFace           * pFace    = NULL;
  SmBSplineSurface * pSurface = NULL;
  SmAxis2Placement   sRefFrame( rRefFrame.GetOrigin() , rRefFrame.GetXAxis(), rRefFrame.GetYAxis() );

  // set m_pBrep for changes
  pBrep->m_bEditingEnabled = TRUE;

  // 1. Build Base plane using original ref frame, domain:[u=length, v=width]
  SmPlane::CreateCanonical( *pContext, sRefFrame, pPlane );
  SmPoint2d  sUVMin(0.0,0.0);
  SmPoint2d  sUVMax(dLength,dWidth);
  SmExtent2d sAnalDomain(sUVMin,sUVMax);
  pSurface = pPlane;
  pSurface->AdjustSTEPUVDomain(sAnalDomain);
  pBrep->CreateFaceFromSurface( (SmSurface*)pSurface, pSurface->GetSTEPUVDomain(), pFace );

  // 2. Build Top plane by translating ref frame to top, domain:[same as BaseFace]
  SmVector3d sZAxis = sRefFrame.GetZAxis();
  sZAxis.Unitize();
  sZAxis = sZAxis * dHeight;
  sRefFrame.Translate( sZAxis );
  SmPlane::CreateCanonical( *pContext, sRefFrame, pPlane );
  pSurface = pPlane;
  pSurface->AdjustSTEPUVDomain(sAnalDomain);
  pBrep->CreateFaceFromSurface( (SmSurface*)pSurface, pSurface->GetSTEPUVDomain(), pFace );


  // 3. Build Right plane by flipping ref frame to Y-Z plane, domain:[u=width, v=height]
  sRefFrame.SetCanonical( rRefFrame.GetOrigin(), rRefFrame.GetYAxis(), rRefFrame.GetZAxis() ); 
  SmPlane::CreateCanonical( *pContext, sRefFrame, pPlane );
  pSurface = pPlane;
  sUVMax.Set( dWidth, dHeight );
  sAnalDomain.SetMinMax(sUVMin,sUVMax);
  pSurface->AdjustSTEPUVDomain(sAnalDomain);
  pBrep->CreateFaceFromSurface( (SmSurface*)pSurface, pSurface->GetSTEPUVDomain(), pFace );

  // 4. Build Left plane by translating ref frame to opposite side, domain:[same as Right plane]
  sZAxis = sRefFrame.GetZAxis();
  sZAxis.Unitize();
  sZAxis = sZAxis * dLength;
  sRefFrame.Translate( sZAxis );
  SmPlane::CreateCanonical( *pContext, sRefFrame, pPlane );
  pSurface = pPlane;
  pSurface->AdjustSTEPUVDomain(sAnalDomain);
  pBrep->CreateFaceFromSurface( (SmSurface*)pSurface, pSurface->GetSTEPUVDomain(), pFace );


  // 5. Build Front plane by flipping ref frame to X-Z plane, domain:[u=length, v=height]
  sRefFrame.SetCanonical( rRefFrame.GetOrigin() , rRefFrame.GetXAxis(), rRefFrame.GetZAxis() ); 
  SmPlane::CreateCanonical( *pContext, sRefFrame, pPlane );
  pSurface = pPlane;
  sUVMax.Set( dLength, dHeight );
  sAnalDomain.SetMinMax(sUVMin,sUVMax);
  pSurface->AdjustSTEPUVDomain(sAnalDomain);
  pBrep->CreateFaceFromSurface( (SmSurface*)pSurface, pSurface->GetSTEPUVDomain(), pFace );

  // 6. Build Back plane by translating ref frame to opposite side, domain:[same as Front plane]
  sZAxis = sRefFrame.GetZAxis();
  sZAxis.Unitize();
  sZAxis = sZAxis * -dWidth;
  sRefFrame.Translate( sZAxis );
  SmPlane::CreateCanonical( *pContext, sRefFrame, pPlane );
  pSurface = pPlane;
  pSurface->AdjustSTEPUVDomain(sAnalDomain);
  pBrep->CreateFaceFromSurface( (SmSurface*)pSurface, pSurface->GetSTEPUVDomain(), pFace );

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if(bDebugMe)
    {
      SM_DUMP_AND_ASSERT_VALID(pBrep) ;

      ULONG di ;
      SmTArray<SmEdge*>    sEdges ;
      SmTArray<SmEdgeuse*> sEdgeuses ; 
      pBrep->GetEdges(sEdges) ;
      pBrep->GetEdgeuses(sEdgeuses) ;

      for(di=0;di<sEdgeuses.GetSize();di++)
        {
          SmEdgeuse      * pEdgeuse = sEdgeuses[di] ;
          SmBSplineCurve * pUVTrimCurve = pEdgeuse->GetUVTrimCurvePointer() ;
          if(pUVTrimCurve) 
            { SM_DUMP_AND_ASSERT_VALID(pUVTrimCurve) ; }
        }

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // build the topology connections between the 6 new faces
    pBrep->StitchAndOrient();

#ifdef SM_DEBUG_CODE
  if(bDebugMe)
    {
      SM_DUMP_AND_ASSERT_VALID(pBrep) ;

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // all done
  pBrep->m_bEditingEnabled = FALSE;
  return( SM_SUCCESS );

} // end SmPrimitiveCreation::CreateBox

/*******************************************************************//**
PURPOSE: Create a cylindrical box solid in the given region of the Brep.
            
NOTES: The 6-sided box is a piece of a thick walled cylinder centered on
       the RefFrame z axis and trimed by top and bot planes and
       start end planes.

       The cylindrical box has two cylindrical faces called the inside
       and outside faces whose radii are the inside and outside
       radii of the infinite thick walled cylinder.

       The Bot and Top planes bound the length of the cylindrical box
       and are perpendicular to the Z axis. The BotPlane contains the
       RefFrame.Origin.  The Top plane contains the point 
       RefFrame.Origin + Length * RefFrame.ZAxis.

       The Start and End planes radiate radially from the RefFrame Z axis
       (like pages radiating from a book's spine) and bound the extent of 
       the cylindrical box.  The Start plane contains the RefFrame Z axis
       and the ray at StartAngDeg from the RefFrame x Axis in the XY plane.
       The End plane also contains the RefFrame Z axis and the ray at EndAngDeg
       from the RefFrame x axis in the XY plane.

  If the vertices/edges of the cylindrical box happen to coincide
  with any vertices/edges of the Brep, they will be 
  stitched together (cf SmStitch).

  The Brep is assumed to have a valid tolerance set.

  For now, the region must be the infinite region of the 
  Brep (limitation of SmStitch).
***********************************************************************/
SmStatus SmPrimitiveCreation::CreateCylindricalBox
 (double                   dLength,        // in : length measured along Z Axis starting at RefFram Origin, [greater than zero]
  double                   dInsideRadius,  // in : Inside Radius measured from Z Axis, [greater than zero]
  double                   dOutsideRadius, // in : Outside Radius measured from Z Axis, [greater than InsideRadius]
  double                   dStartAngDeg,   // in : Start AngDeg measured from X Axis in YX plane, [-360 to 360]
  double                   dEndAngDeg,     // in : End   AngDeg measured from X Axis in YX plane, [EndAng-StartAng in range:(0 to 360)]
  const SmAxis2Placement & rRefFrame)      // in : Z Axis  = Cylinder centers
                                           //      Origin  = Bot of CylBox
                                           //      XY Axes = Plane of Start and End angles measured from X Axis
{
  // check input
  SmBoolean bLength = dLength > SM_EFF_ZERO;
  AERN_MSG( bLength, SM_ERR_INVALID_INPUT,                                                   _T("SmPrimitiveCreation::CreateCylindricalBox, Length not greater than zero - box not made")) ;
  
  SmBoolean bInsideRadius = dInsideRadius > SM_EFF_ZERO;
  AERN_MSG( bInsideRadius, SM_ERR_INVALID_INPUT,                                             _T("SmPrimitiveCreation::CreateCylindricalBox, InsideRadius not greater than zero - box not made")) ;
  
  SmBoolean bOutsideRadius = dOutsideRadius > dInsideRadius + SM_EFF_ZERO;
  AERN_MSG( bOutsideRadius, SM_ERR_INVALID_INPUT,                                            _T("SmPrimitiveCreation::CreateCylindricalBox, OutsideRadius not greater than InsideRadius - box not made")) ;
  
  AERN_MSG( SM_IS_CONTAINED_TO_TOL(dStartAngDeg, -360, 360, 0), SM_ERR_INVALID_INPUT,         _T("SmPrimitiveCreation::CreateCylindricalBox, StartAngDeg not in range:[-360 360] - box not made")) ;
  
  SmBoolean bAngDeg = dEndAngDeg > dStartAngDeg + SM_EFF_ZERO;
  AERN_MSG( bAngDeg, SM_ERR_INVALID_INPUT,                                                   _T( "SmPrimitiveCreation::CreateCylindricalBox, EndAngDeg not greater than StartAng - box not made" ) );
  
  AERN_MSG( SM_IS_CONTAINED_TO_TOL(dEndAngDeg-dStartAngDeg, 0, 360, 0), SM_ERR_INVALID_INPUT, _T("SmPrimitiveCreation::CreateCylindricalBox, EndAngDeg-StartAng not in range:[0 360] - box not made")) ;

  // locals
  SmBrep           * pBrep     = m_pRegion->GetBrep();
  const SmContext  * pContext  = pBrep->GetContext();
  SmPlane          * pPlane    = NULL ;
  SmCylinder       * pCylinder    = NULL ;
  SmAxis2Placement   sRefFrame(rRefFrame) ;
  SmRegion         * pNewRegion = NULL;
  SmShell          * pNewShell = NULL;
  SmFace           * pNewFace = NULL;
  // SmBoolean          bRunHeal = FALSE ; 

  // locals for MakeFaceWithCurves
  SmTArray<ULONG>        sCurveLoops ; sCurveLoops.Add(4) ;  
  SmTArray<SmCurve*>     sCurves ; 
  SmTArray<SmPoint3d>    sLoopPoints ;
  SmTArray<SmOrientType> sCurveOrientations ; 
  sCurveOrientations.Add(SM_OT_SAME) ;                       
  sCurveOrientations.Add(SM_OT_SAME) ;                       
  sCurveOrientations.Add(SM_OT_SAME) ;                       
  sCurveOrientations.Add(SM_OT_SAME) ;
  
  // geometry locals
  double dCosStartAng = smos_Cosine(SM_DEG2RAD(dStartAngDeg)) ;
  double dSinStartAng = smos_Sine(SM_DEG2RAD(dStartAngDeg)) ;
     
  double dCosMidAng   = smos_Cosine(SM_DEG2RAD((dStartAngDeg + dEndAngDeg)/2.0)) ;
  double dSinMidAng   = smos_Sine(SM_DEG2RAD((dStartAngDeg + dEndAngDeg)/2.0)) ;
     
  double dCosEndAng   = smos_Cosine(SM_DEG2RAD(dEndAngDeg)) ;
  double dSinEndAng   = smos_Sine(SM_DEG2RAD(dEndAngDeg)) ;
  
  SmVector3d sLengthVec = dLength * rRefFrame.GetZAxis() ;    
  SmVector3d sStartVec  = dCosStartAng * rRefFrame.GetXAxisRef() + dSinStartAng * rRefFrame.GetYAxisRef() ; 
  SmVector3d sMidVec    = dCosMidAng   * rRefFrame.GetXAxisRef() + dSinMidAng   * rRefFrame.GetYAxisRef() ; 
  SmVector3d sEndVec    = dCosEndAng   * rRefFrame.GetXAxisRef() + dSinEndAng   * rRefFrame.GetYAxisRef() ; 

  // corner and arc mid points of the BotPlane boundary   
  SmPoint3d sBotInsideStartPoint  = rRefFrame.GetOriginRef() + dInsideRadius  * sStartVec ; 
  SmPoint3d sBotInsideMidPoint    = rRefFrame.GetOriginRef() + dInsideRadius  * sMidVec ; 
  SmPoint3d sBotInsideEndPoint    = rRefFrame.GetOriginRef() + dInsideRadius  * sEndVec ; 
                                                           
  SmPoint3d sBotOutsideStartPoint = rRefFrame.GetOriginRef() + dOutsideRadius * sStartVec ; 
  SmPoint3d sBotOutsideMidPoint   = rRefFrame.GetOriginRef() + dOutsideRadius * sMidVec ; 
  SmPoint3d sBotOutsideEndPoint   = rRefFrame.GetOriginRef() + dOutsideRadius * sEndVec ; 

  // corner and arc mid points of the TopPlane boundary   
  SmPoint3d sTopInsideStartPoint  = sBotInsideStartPoint  + sLengthVec ; 
  SmPoint3d sTopInsideMidPoint    = sBotInsideMidPoint    + sLengthVec ; 
  SmPoint3d sTopInsideEndPoint    = sBotInsideEndPoint    + sLengthVec ; 
                                      
  SmPoint3d sTopOutsideStartPoint = sBotOutsideStartPoint + sLengthVec ; 
  SmPoint3d sTopOutsideMidPoint   = sBotOutsideMidPoint   + sLengthVec ; 
  SmPoint3d sTopOutsideEndPoint   = sBotOutsideEndPoint   + sLengthVec ; 

  // set m_pBrep for changes
  pBrep->m_bEditingEnabled = TRUE;

  // 1. Make and Add BotFace to Brep 
  
  // Build Bot Plane positioned through original ref frame, domain:[-dOutsideRadius, -dOutsideRadius, dOutsideRadius, dOutsideRadius]
  SmPlane::CreateCanonical( *pContext, sRefFrame, pPlane );
  SmExtent2d sSurfUVDomain(-dOutsideRadius, -dOutsideRadius, dOutsideRadius, dOutsideRadius) ;
  pPlane->AdjustSTEPUVDomain(sSurfUVDomain);

  // curves of the Bot Plane
  SmLine   * pBotStartLine     = new (*pContext) SmLine  (sBotInsideStartPoint,  sBotOutsideStartPoint, 3, pContext) ;
  SmCircle * pBotOutsideCircle = new (*pContext) SmCircle(sBotOutsideStartPoint, sBotOutsideMidPoint, sBotOutsideEndPoint) ;
  SmLine   * pBotEndLine       = new (*pContext) SmLine  (sBotOutsideEndPoint,   sBotInsideEndPoint, 3, pContext) ;
  SmCircle * pBotInsideCircle  = new (*pContext) SmCircle(sBotInsideEndPoint,    sBotInsideMidPoint, sBotInsideStartPoint) ;

  // assemble loop of bot face trim curves
  sCurves.ReSet() ;
  sCurves.Add(pBotStartLine    ) ; 
  sCurves.Add(pBotOutsideCircle) ; 
  sCurves.Add(pBotEndLine      ) ;    
  sCurves.Add(pBotInsideCircle ) ; 

  // Make Bot Plane Face in Brep - trim curves now owned by new Face - don't delete or use them again                   
  pBrep->MakeFaceWithCurves(m_pRegion,          // in : region to contain new topology objects
                            sCurveLoops,        // in : 1 entry per loop, value = loop edge count, 1st entry=outer loop
                            &sCurves,           // in : opt ordered 3d trimming curves assigned to loops per sLoopEUCounts, curves owned by new face
                            NULL,               // in : opt ordered 2d trimming curves assigned to loops per sLoopEUCounts, curves owned by new face
                            sCurveOrientations, // in : associated orients for each trimming curve, SM_OT_SAME or SM_OT_OPPOSITE
                            sLoopPoints,        // in : Point positions to build SmVertex VertexLoops
                            pPlane,             // in : new face->Surface
                            sSurfUVDomain,      // in : domain of Surface used by face
                            SM_OT_SAME,         // in : Surface orient, oneof SM_OT_SAME or SM_OT_OPPOSITE
                            pNewRegion,         // out: New region if any. NULL when building trimmed surfaces, may be NotNULL for solids.
                            pNewShell,          // out: New shell if any.  Trimmed surfaces always create a new shell.
                            pNewFace) ;         // out: the new face
                       //     bRunHeal) ;         // in : TRUE = Run Healer on new Face - may edit the Face data to fix problems
                                                  //      FALSE= don't

  // 2. Make and Add TopFace to Brep 
  
  // Build Top Plane ref frame translated by length in Z direction, domain:[-dOutsideRadius, -dOutsideRadius, dOutsideRadius, dOutsideRadius]
  sRefFrame.Translate(sLengthVec) ;
  SmPlane::CreateCanonical( *pContext, sRefFrame, pPlane );
  sSurfUVDomain.SetMinMax(-dOutsideRadius, -dOutsideRadius, dOutsideRadius, dOutsideRadius) ;
  pPlane->AdjustSTEPUVDomain(sSurfUVDomain);

  // curves of the Top Plane
  SmLine   * pTopStartLine     = new (*pContext) SmLine(sTopInsideStartPoint, sTopOutsideStartPoint, 3, pContext) ;
  SmCircle * pTopOutsideCircle = new (*pContext) SmCircle(sTopOutsideStartPoint, sTopOutsideMidPoint, sTopOutsideEndPoint) ;
  SmLine   * pTopEndLine       = new (*pContext) SmLine(sTopOutsideEndPoint, sTopInsideEndPoint, 3, pContext) ;
  SmCircle * pTopInsideCircle  = new (*pContext) SmCircle(sTopInsideEndPoint, sTopInsideMidPoint, sTopInsideStartPoint) ;

  // assemble loop of top face trim curves
  sCurves.ReSet() ;
  sCurves.Add(pTopStartLine    ) ; 
  sCurves.Add(pTopOutsideCircle) ; 
  sCurves.Add(pTopEndLine      ) ;    
  sCurves.Add(pTopInsideCircle ) ; 

  // Make Bot Plane Face in Brep - trim curves now owned by new Face - don't delete or use them again                   
  pBrep->MakeFaceWithCurves(m_pRegion,          // in : region to contain new topology objects
                            sCurveLoops,        // in : 1 entry per loop, value = loop edge count, 1st entry=outer loop
                            &sCurves,           // in : opt ordered 3d trimming curves assigned to loops per sLoopEUCounts, curves owned by new face
                            NULL,               // in : opt ordered 2d trimming curves assigned to loops per sLoopEUCounts, curves owned by new face
                            sCurveOrientations, // in : associated orients for each trimming curve, SM_OT_SAME or SM_OT_OPPOSITE
                            sLoopPoints,        // in : Point positions to build SmVertex VertexLoops
                            pPlane,             // in : new face->Surface
                            sSurfUVDomain,      // in : domain of Surface used by face
                            SM_OT_SAME,         // in : Surface orient, oneof SM_OT_SAME or SM_OT_OPPOSITE
                            pNewRegion,         // out: New region if any. NULL when building trimmed surfaces, may be NotNULL for solids.
                            pNewShell,          // out: New shell if any.  Trimmed surfaces always create a new shell.
                            pNewFace) ;         // out: the new face
                       //     bRunHeal) ;         // in : TRUE = Run Healer on new Face - may edit the Face data to fix problems
                       //                         //      FALSE= don't

  // 3. Make and Add StartFace to Brep - It's naturally trimmed, so call CreateFaceFromSurface() 
  
  // Build Start Plane positioned:[orig, StartVec, ZVec], domain:[InsideRad, 0, OutSideRad, Length]
  sRefFrame.SetCanonical(rRefFrame.GetOriginRef(), sStartVec, rRefFrame.GetZAxis()) ; 
  SmPlane::CreateCanonical( *pContext, sRefFrame, pPlane );
  sSurfUVDomain.SetMinMax(dInsideRadius, 0, dOutsideRadius, dLength) ;
  pPlane->AdjustSTEPUVDomain(sSurfUVDomain);
  pBrep->CreateFaceFromSurface( pPlane, pPlane->GetNaturalUVDomain(), pNewFace );

  // 4. Make and Add EndFace to Brep - It's naturally trimmed, so call CreateFaceFromSurface() 
  
  // Build End Plane positioned:[orig, EndVec, ZVec], domain:[InsideRad, 0, OutSideRad, Length]
  sRefFrame.SetCanonical(rRefFrame.GetOriginRef(), sEndVec, rRefFrame.GetZAxis()) ; 
  SmPlane::CreateCanonical( *pContext, sRefFrame, pPlane );
  sSurfUVDomain.SetMinMax(dInsideRadius, 0, dOutsideRadius, dLength) ;
  pPlane->AdjustSTEPUVDomain(sSurfUVDomain);
  pBrep->CreateFaceFromSurface( pPlane, pPlane->GetNaturalUVDomain(), pNewFace );

// 5. Make and Add InsideFace to Brep - It's naturally trimmed, so call CreateFaceFromSurface() 
  
  // Build End Plane positioned:[orig, EndVec, ZVec], domain:[dStartAngDeg, 0, dEndAngDeg, dLength]
  SmCylinder::CreateCanonical(*pContext, rRefFrame, dInsideRadius, pCylinder) ; 
  sSurfUVDomain.SetMinMax(dStartAngDeg, 0, dEndAngDeg, dLength) ;
  pCylinder->AdjustSTEPUVDomain(sSurfUVDomain);
  pBrep->CreateFaceFromSurface( pCylinder, pCylinder->GetNaturalUVDomain(), pNewFace );

  // 6. Make and Add OutsideFace to Brep - It's naturally trimmed, so call CreateFaceFromSurface() 
  
  // Build End Plane positioned:[orig, EndVec, ZVec], domain:[dStartAngDeg, 0, dEndAngDeg, dLength]
  SmCylinder::CreateCanonical(*pContext, rRefFrame, dOutsideRadius, pCylinder) ; 
  sSurfUVDomain.SetMinMax(dStartAngDeg, 0, dEndAngDeg, dLength) ;
  pCylinder->AdjustSTEPUVDomain(sSurfUVDomain);
  pBrep->CreateFaceFromSurface( pCylinder, pCylinder->GetNaturalUVDomain(), pNewFace );

  // build the topology connections between the 6 new faces
  pBrep->StitchAndOrient() ;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if(bDebugMe)
    {
      ULONG di ;
      SmTArray<SmFace*>    sFaces ;    pBrep->GetFaces(sFaces) ; 

      SM_DUMP_AND_ASSERT_VALID(pBrep) ;

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,3, 0,0,0) ; for(di=0;di<sFaces.GetSize();di++) 
                                    { if(sFaces[di]) sFaces[di]->DrawUV(7,7,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ; }
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // all done
  pBrep->m_bEditingEnabled = FALSE;
  return( SM_SUCCESS );

} // end SmPrimitiveCreation::CreateCylindricalBox

/*******************************************************************//**
PURPOSE: Creates a conic solid in the given region of the Brep.
            
NOTES: See SmPrimitiveCreation::CreateCone
                This creates only full cones
***********************************************************************/
SmStatus SmPrimitiveCreation::CreateCone
 (double                   dHeight,        // in : height of result
  double                   dBaseRadius,    // in : radius of base
  double                   dTopRadius,     // in : cylinder if same as dBaseRadius. cone if different
  const SmAxis2Placement & rRefFrame)      // in : position and orientation of result
{

  SmBrep* pBrep = m_pRegion->GetBrep();
  // Leave edit mode enabled on exit: callers rely on it to keep editing the cone.
  pBrep->m_bEditingEnabled = TRUE;

  const SmContext* cpContext = pBrep->GetContext();

  // Build cone surface. Degenerate input -> NULL surface; don't dereference it.
  SmCone* pConeSrf = NULL;
  SER(SmCone::CreateCanonical( *cpContext, rRefFrame, dBaseRadius, dTopRadius, dHeight, pConeSrf ));
  NER(pConeSrf);
  SmObjDelete sConeCleanup(pConeSrf);   // free the surface if face creation fails

  SmFace* newFace = NULL;
  SER(pBrep->CreateFaceFromSurface( (SmSurface*)pConeSrf, pConeSrf->GetNaturalUVDomain(), newFace ));
  sConeCleanup.Clear();                 // brep now owns the surface

  // Build Caps
  SmExtent2d sDomain = pConeSrf->GetNaturalUVDomain();

  SmBSplineCurve* pBaseEdge = NULL;
  SER(pConeSrf->CreateIsoParametricCurve(*cpContext, 
                                     SM_SP_V, 
                                     sDomain.GetMin().y, 
                                     0.0, 
                                     pBaseEdge ));
  SmBSplineCurve* pTopEdge = NULL;
  SER(pConeSrf->CreateIsoParametricCurve(*cpContext, 
                                      SM_SP_V, 
                                      sDomain.GetMax().y, 
                                      0.0, 
                                      pTopEdge ));
 
  double dBrepTol = pBrep->GetTolerance();

  // make base face
  SmFace* pBaseFace = NULL;
  if( dBaseRadius > dBrepTol ) 
    {
      SmTArray <SmCurve*> pCurves;
      pCurves.Add(pBaseEdge);    
      SER(pBrep->CreatePlanarFaceWith3DCurves(m_pRegion,   // in : region to contain newFace - not checked
                                                       //      NULL for infinite region.
                                          pCurves,     // in : array of planar curves to bound newFace - curves used by NewEdges
                                          dBrepTol,    // in : 3d Distance for planarity checks
                                          pBaseFace )); // out: new face
    }

  // make top face
  SmFace* pTopFace = NULL;
  if( dTopRadius > dBrepTol ) 
    {
      SmTArray <SmCurve*> pCurves;
      pCurves.Add(pTopEdge);     
      SER(pBrep->CreatePlanarFaceWith3DCurves(m_pRegion,  // in : region to contain newFace - not checked
                                                      //      NULL for infinite region.
                                          pCurves,    // in : array of planar curves to bound newFace - curves used by NewEdges
                                          dBrepTol,   // in : 3d Distance for planarity checks
                                          pTopFace )); // out: new face
    }

  // if any caps then stitch faces together to form solid
  if( pBaseFace || pTopFace ) {
      SER(pBrep->StitchAndOrient());
  }

  return( SM_SUCCESS );
} // end SmPrimitiveCreation::CreateCone

/*******************************************************************//**
PURPOSE: Creates a conic solid in the given region of the Brep.
            
NOTES: The center of the planar circular face with radius 
                dBaseRadius is in the origin of the frame, the other center
                is in dir Z axis at distance dHeight.
            
                If the vertices/edges of the cone happen to coincide
                with any vertices/edges of the Brep, they will be 
                stitched together (cf SmStitch)

                The Brep is assumed to have a valid tolerance set.

                For now, the region must be the infinite region of the 
                Brep (limitation of SmStitch).
***********************************************************************/
SmStatus SmPrimitiveCreation::CreateCone
 (double dHeight,                    // in : height of result
  double dBaseRadius,                // in : radius of base
  double dTopRadius,                 // in : cylinder if same as dBaseRadius. cone if different
  double dStartAngleDeg,             // in : measured in the XY plane from the X axis
  double dEndAngleDeg,               // in : measured in the XY plane from the X axis
  const SmAxis2Placement& rRefFrame) // in : position and orientation of result
{
  // empty entity maps - m_vMapToSame, m_vMapToHigher, m_vMapFromSame, and m_vMapFromLower
  InitEntityMaps();

  if (dHeight < SM_EFF_ZERO) {
      SER(SM_ERR);
  }

  // Check for special cases
  // if full 360 degree sweep then create analytic
  // if zero sweep, then fail
  double dRotAngle = (dEndAngleDeg - dStartAngleDeg);
  if(fabs(dRotAngle) < SM_EFF_ZERO) {
      SER(SM_ERR);
  }

  SmBoolean bFullRot = 
      (smos_Fabs(dRotAngle - 360.0) < 360.0*SM_EFF_ZERO) ? TRUE : FALSE;

  #ifdef USE_NEW_ANALYTICS
  if( bFullRot ) {
      return( CreateCone( dHeight, dBaseRadius, dTopRadius, rRefFrame ) );
  }
  #endif // USE_NEW_ANALYTICS

  // Construct partial cone
  SmBrep* pBrep = m_pRegion->GetBrep();
  pBrep->m_bEditingEnabled = TRUE;

  const SmContext* cpContext = pBrep->GetContext();

  const SmPoint3d &rOrigin = rRefFrame.GetOriginRef();

  // The axis of the cone is the Z axis of the Frame:
  SmVector3d sAxis = rRefFrame.GetZAxis() ;

  double dStartAngleRad = dStartAngleDeg * SM_PI / 180.0; 

  SmAxis2Placement sRefFrame(rRefFrame);
  sRefFrame.RotateAboutAxis(dStartAngleRad, sAxis);

  const SmVector3d &rXAxis = sRefFrame.GetXAxisRef() ;

  SmPoint3d sTopBasePt( rOrigin    + dHeight     * sAxis );
  SmPoint3d sStartPt  ( rOrigin    + dBaseRadius * rXAxis );
  SmPoint3d sEndPt    ( sTopBasePt + dTopRadius  * rXAxis );

  SmTArray<SmCurve*> a3DCurves;
  SmObjsDelete<SmCurve*> sClean3d(&a3DCurves);

  double dTol = SM_EFF_ZERO * (1.0 + sEndPt.GetMaxDimension() +
      sStartPt.GetMaxDimension() );

  if (sEndPt.DistanceBetween(sStartPt) > dTol) {
      SmBSplineCurve *pLine1 = NULL;
      SER(SmBSplineCurve::CreateLineSegment(*cpContext,3,
              sStartPt, sEndPt, pLine1));
      a3DCurves.Add(pLine1);
  }
  else { 
      SER(SM_ERR); // Zero length side of a cone is not legal
  }

  if (rOrigin.DistanceBetween(sStartPt) > dTol) {
      SmBSplineCurve *pLine2 = NULL ;
      SER(SmBSplineCurve::CreateLineSegment(*cpContext,3,
          rOrigin, sStartPt, pLine2));
      a3DCurves.Add(pLine2);
  }

  if (rOrigin.DistanceBetween(sTopBasePt) > dTol) {
      SmBSplineCurve *pLine3 = NULL;
      SER(SmBSplineCurve::CreateLineSegment(*cpContext,3,
          rOrigin, sTopBasePt, pLine3));
      a3DCurves.Add(pLine3);
  }
  else { 
      SER(SM_ERR); // Zero length side of a cone is not legal
  }

  if (sEndPt.DistanceBetween(sTopBasePt) > dTol) {
      SmBSplineCurve *pLine4 = NULL;
      SER(SmBSplineCurve::CreateLineSegment(*cpContext,3,
                  sEndPt, sTopBasePt, pLine4));
      a3DCurves.Add(pLine4);
  }

  if (a3DCurves.GetSize() < 3) {
      SER(SM_ERR); // Some problem here with not enough curves
  }


  sClean3d.Clear();

  SER(CreateRotationalSweep(a3DCurves, rOrigin, sAxis, 
      dRotAngle, 1, bFullRot ? FALSE : TRUE));


  pBrep->m_bEditingEnabled = FALSE;
  return SM_SUCCESS;

} // end SmPrimitiveCreation::CreateCone

/*******************************************************************//**
PURPOSE: Creates a spheric solid in the given region of the Brep.
            
NOTES: See SmPrimitiveCreation::CreateSphere
                For full sphere only at this time
***********************************************************************/
SmStatus SmPrimitiveCreation::CreateSphere
 (double dRadius,                     // in : radius of sphere
  const SmAxis2Placement& rRefFrame ) // in : position and orientation of result
{
  // empty entity maps - m_vMapToSame, m_vMapToHigher, m_vMapFromSame, and m_vMapFromLower
  InitEntityMaps();

  SmBrep* pBrep = m_pRegion->GetBrep();
  // RAII: enable edit mode; auto-restored on every return path.
  SmTemporaryChangeValue<SmBoolean> sEditing(pBrep->m_bEditingEnabled, TRUE);

  const SmContext* cpContext = pBrep->GetContext();

  // Degenerate input -> NULL surface; don't dereference it.
  SmSphere* pSphere = NULL;
  SER(SmSphere::CreateCanonical( *cpContext, rRefFrame, dRadius, pSphere ));
  NER(pSphere);
  SmObjDelete sSphereCleanup(pSphere);   // free the surface if face creation fails

  SmFace* newFace = NULL;
  SER(pBrep->CreateFaceFromSurface( (SmSurface*)pSphere, pSphere->GetNaturalUVDomain(), newFace ));
  sSphereCleanup.Clear();                // brep now owns the surface

  return SM_SUCCESS;
} // end SmPrimitiveCreation::CreateSphere

/*******************************************************************//**
PURPOSE: Creates a spheric solid in the given region of the Brep.
            
NOTES:
   The center of the sphere is in the origin of the frame.

   If the vertices/edges of the sphere happen to coincide with any
   vertices/edges of the Brep, they will be stitched together (cf SmStitch)

   The Brep is assumed to have a valid tolerance set.

   For now, the region must be the infinite region of the Brep
   (limitation of SmStitch).
***********************************************************************/
SmStatus SmPrimitiveCreation::CreateSphere
 (double dRadius,                    // in : radius of sphere
  double dStartAngleDeg,             // in : measured in the XY plane from the X axis
  double dEndAngleDeg,               // in : measured in the XY plane from the X axis
  const SmAxis2Placement& rRefFrame) // in : position and orientation of result
{
  // empty entity maps - m_vMapToSame, m_vMapToHigher, m_vMapFromSame, and m_vMapFromLower
  InitEntityMaps();

  double dRotAngle = (dEndAngleDeg - dStartAngleDeg);
  SmBoolean bFullRot = 
      (smos_Fabs(dRotAngle - 360.0) < 360.0*SM_EFF_ZERO) ? TRUE : FALSE;

  #ifdef USE_NEW_ANALYTICS
  if( bFullRot ) {
      return( CreateSphere( dRadius, rRefFrame ) );
  }
  #endif // USE_NEW_ANALYTICS

  /*
  // Construct partial sphere
  //        
  //             sEndAxis
  //            /
  //           /
  //          /  
  //         / 
  //       Z      DeltaAngle 
  //       | \ 
  //       |  \ 
  //       |   \  <-------------------This is the top view of a circle 
  //       |    \ 
  //       V     sStartAxis 
  //       X  dStartAngle 
  //
  */

  SmBrep* pBrep = m_pRegion->GetBrep();
  pBrep->m_bEditingEnabled = TRUE;

  //. convert from degrees to radians
  double dStartAngleRad = dStartAngleDeg * SM_PI / 180.0; 

  const SmPoint3d &rOrigin = rRefFrame.GetOriginRef() ;
  SmVector3d sAxis( rRefFrame.GetZAxis() );

  SmAxis2Placement sRefFrame1(rRefFrame);
  sRefFrame1.RotateAboutAxis(dStartAngleRad, sAxis);
  //const SmVector3d &rStartAxis = sRefFrame1.GetXAxisRef() ;
  const SmVector3d &rYRF1      = sRefFrame1.GetYAxisRef();

  SmPoint3d sStartPt( rOrigin + dRadius * sAxis );
  SmPoint3d sEndPt  ( rOrigin - dRadius * sAxis);

  const SmContext* cpContext = pBrep->GetContext();
  SmBSplineCurve *pLine1 = NULL ;
  SER(SmBSplineCurve::CreateLineSegment(*cpContext,3,
              sStartPt, sEndPt, pLine1));


  // We need a frame for the circle: 
  // its X is -Axis, its Z is is the Y of RefFrame1
  SmAxis2Placement sRefFrame3;
  SmStatus sStat = 
      sRefFrame3.SetCanonical(rOrigin, -sAxis, rYRF1 * sAxis); 
  if (sStat != SM_SUCCESS) { SER(SM_ERR); }

  SmNurbCircleParam eParameterization = SM_CO_QUADRATIC;  
  // The circle generator can cope all angles.
  // If there is need, use SM_CO_QUINTIC.

  SmBSplineCurve *pCircle = NULL ;
  SER(SmBSplineCurve::CreateCircleSegment(*cpContext,
      3,         // lDimensionOfResult
      sRefFrame3, // ReferenceFrame
      dRadius,
      0.0,       // dStartAngle 
      180.0,     // dEndAngle 
      eParameterization, // degree of nurbc
      pCircle));

  SmTArray<SmCurve*> a3DCurves;
  a3DCurves.Add(pLine1);
  a3DCurves.Add(pCircle);

  SER(CreateRotationalSweep(a3DCurves, rOrigin, sAxis, 
      dRotAngle, 1, bFullRot ? FALSE : TRUE));

  pBrep->m_bEditingEnabled = FALSE;
  return SM_SUCCESS;
} // end SmPrimitiveCreation::CreateSphere

/*******************************************************************//**
PURPOSE: Creates a toric solid in the given region of the Brep.
            
NOTES: See SmPrimitiveCreation::CreateTorus.
       For full torus only at this time.
***********************************************************************/
SmStatus SmPrimitiveCreation::CreateTorus
 (double dMajorRadius,               // in : major radius of torus
  double dMinorRadius,               // in : minor radius of torus
  const SmAxis2Placement& rRefFrame) // in : position and orientation of result
{
  // empty entity maps - m_vMapToSame, m_vMapToHigher, m_vMapFromSame, and m_vMapFromLower
  InitEntityMaps();

  // A minor >= major ("spindle") torus is a valid representation for some trimmed domains, so it is
  // not rejected here; SmTorus::CreateCanonical guards genuinely degenerate (non-positive) radii and
  // fails cleanly, which the callers below turn into a graceful error rather than a crash.

  SmBrep* pBrep = m_pRegion->GetBrep();
  // RAII: enable edit mode; auto-restored on every return path.
  SmTemporaryChangeValue<SmBoolean> sEditing(pBrep->m_bEditingEnabled, TRUE);

  const SmContext* cpContext = pBrep->GetContext();

  // Degenerate radii -> NULL surface; don't dereference it.
  SmTorus* pTorus = NULL;
  SER(SmTorus::CreateCanonical( *cpContext, rRefFrame, dMajorRadius, dMinorRadius, pTorus ));
  NER(pTorus);
  SmObjDelete sTorusCleanup(pTorus);     // free the surface if face creation fails

  SmFace* newFace = NULL;
  SER(pBrep->CreateFaceFromSurface( (SmSurface*)pTorus, pTorus->GetNaturalUVDomain(), newFace ));
  sTorusCleanup.Clear();                 // brep now owns the surface

  return SM_SUCCESS;

} // end SmPrimitiveCreation::CreateTorus

/*******************************************************************//**
PURPOSE: Creates a toric solid in the given region of the Brep.
            
NOTES:
   The center of the torus is in the origin of the frame.

   If the vertices/edges of the torus happen to coincide with any
   vertices/edges of the Brep, they will be stitched together (cf SmStitch)

   The Brep is assumed to have a valid tolerance set.

   For now, the region must be the infinite region of the Brep
   (limitation of SmStitch).
***********************************************************************/
SmStatus SmPrimitiveCreation::CreateTorus
 (double dMajorRadius,               // in : major radius of torus
  double dMinorRadius,               // in : minor radius of torus
  double dStartAngleDeg,             // in : measured in the XY plane from the X axis
  double dEndAngleDeg,               // in : measured in the XY plane from the X axis
  const SmAxis2Placement& rRefFrame) // in : position and orientation of result
{
  // empty entity maps - m_vMapToSame, m_vMapToHigher, m_vMapFromSame, and m_vMapFromLower
  InitEntityMaps();

   double dRotAngle = (dEndAngleDeg - dStartAngleDeg);
  SmBoolean bFullRot = 
      (smos_Fabs(dRotAngle - 360.0) < 360.0*SM_EFF_ZERO) ? TRUE : FALSE;

  #ifdef USE_NEW_ANALYTICS
  if( bFullRot ) {
      return( CreateTorus( dMajorRadius, dMinorRadius, rRefFrame ) );
  }
  #endif // USE_NEW_ANALYTICS

  /*
  // Construct partial torus
  //        
  //             sEndAxis
  //            /
  //           /
  //          /  
  //         / 
  //       Z      DeltaAngle 
  //       | \ 
  //       |  \ 
  //       |   \ \ <-------------------This is the top view of a circle 
  //       |    \ \ 
  //       V     C \    
  //       X  dStart\Angle  
  //                 \ 
  //           StartAxis 
  */

  // A minor >= major ("spindle") torus is a valid representation for some trimmed domains, so it is
  // not rejected here; SmTorus::CreateCanonical guards genuinely degenerate (non-positive) radii and
  // fails cleanly, which the callers below turn into a graceful error rather than a crash.

  SmBrep* pBrep = m_pRegion->GetBrep();
  pBrep->m_bEditingEnabled = TRUE;

  double dStartAngleRad = dStartAngleDeg * SM_PI / 180.0; 

  SmVector3d sAxis( rRefFrame.GetZAxis() );
  const SmPoint3d &rOrigin = rRefFrame.GetOriginRef() ;
  SmAxis2Placement sRefFrame1(rRefFrame);
  sRefFrame1.RotateAboutAxis(dStartAngleRad, sAxis);
  const SmVector3d &rStartAxis = sRefFrame1.GetXAxisRef() ;
  const SmVector3d &rYRF1      = sRefFrame1.GetYAxisRef() ;

  SmPoint3d sCenter = rOrigin + dMajorRadius * rStartAxis;

  const SmContext* cpContext = pBrep->GetContext();

  SmTArray<SmCurve*> a3DCurves;

  // We need a frame for the full circle: 
  // its x is the -axis, its Z is YRF1

  SmAxis2Placement sRefFrame3;
  SmStatus sStat = 
      sRefFrame3.SetCanonical(sCenter, -sAxis, -rYRF1 * sAxis); 
  if (sStat != SM_SUCCESS) { SER(SM_ERR); }
    
  SmNurbCircleParam eParameterization = SM_CO_QUADRATIC;  
  // The circle generator can cope all angles.
  // If there is need, use SM_CO_QUINTIC.

  // if minor radius = major radius, adjust start and end angle -180 to 180
  // we do this so the singularity ends up at the seam which
  // seems to work better for intersections
  double dStartAngle = 0.0;
  double dEndAngle = 360.0;
  if( dMinorRadius == dMajorRadius ) {
      dStartAngle = -180;
      dEndAngle = 180;
  }

   SmBSplineCurve *pCircle = NULL ;
   SER(SmBSplineCurve::CreateCircleSegment(*cpContext,
       3,         // lDimensionOfResult
       sRefFrame3, // ReferenceFrame
       dMinorRadius,
       dStartAngle,
       dEndAngle, 
       eParameterization, // deg of nurbc
       pCircle));
   a3DCurves.Add(pCircle);

  SER(CreateRotationalSweep(a3DCurves, rOrigin, sAxis, 
      dRotAngle, 1, bFullRot ? FALSE : TRUE));


  pBrep->m_bEditingEnabled = FALSE;
  return SM_SUCCESS;
} // end SmPrimitiveCreation::CreateTorus

/*******************************************************************/ /**
 PURPOSE: Creates a planar face from each of the given curves,
             and extrudes them, possibly into a solid.

 NOTES: This is a static version which makes use of SmSurfOfExtrusion.
        This is only called from SmPrimitiveCreation::CreateLinearSweep.
        See SmPrimitiveCreation::CreateLinearSweep for additional doc.

        If the curves are not closed loops the capping will fail.
 ***********************************************************************/
SmStatus SmPrimitiveCreation::CreateLinearSweep(SmTArray<SmCurve*>& cr3DCurves, // in : Must be coplanar - these curves
                                                                                // are consumed
                                                const SmVector3d& crSweepVec, // in : The sweep vector
                                                double dSweepDist, // in : Sweep = SweepDist * SweepVec
                                                SmBoolean bCapEnds) // in : TRUE = build end caps
{
    // locals
    SmBrep* pBrep = m_pRegion->GetBrep();
    const SmContext* pContext = pBrep->GetContext();
    double dTol = pBrep->GetTolerance();
    SmExtent2d sAnalDomain(0.0, 0.0, 1.0, dSweepDist);
    SmSurface* pExtSurface = NULL;

    SmTemporaryChangeValue<SmBoolean> sDoingBool(SM_CONST_CAST(SmContext*, pContext)->GetDoingBooleanRef(), TRUE);
    // Memory management:
    // This routine consumes (or deletes) the input curves.
    // It does two things: creates the sweep surface and face,
    // then optionally creates both end caps.
    // When creating the sweep surface, each input curve is treated
    // separately: either creates an analytic surface (plane or cylinder),
    // or a general SmSurfOfExtrusion.  The SmSurfOfExtrusion consumes the input
    // curve, but the analytic methods do not.
    // For end capping, the curves are consumed: we could use the input curves
    // directly for the start end cap, but we are required to be copied (and
    // translated) for the end end cap.
    // We could go two ways: either make a copy of each curve that gets
    // consumed in the first part and delete all input curves when done,
    // or keep track of which curves need to be deleted when done.
    // While the first option requires more copying in some cases, it is
    // much more straightforward, so we'll go that route.
    //
    //  Create sweep surface:
    //    For each curve
    //      If the curve is consumed by the surface,
    //        make a copy of it
    //
    //  If end caps:
    //    create start cap from input curves
    //    create end cap from copies
    //  else
    //    delete input curves.
    //
    // We accomplish this by setting up an SmObjsDelete to start,
    // and clearing it if we make end caps.

    SmObjsDelete<SmCurve*> sCleanInputCurves(&cr3DCurves);


#ifdef SM_DEBUG_CODE
    SmBoolean bDebugMe = FALSE;
    // draw inputs
    if (bDebugMe)
    {
        SmPoint3d sBasePoint;
        if (cr3DCurves.GetSize() > 0)
        {
            cr3DCurves[0]->EvaluatePoint(cr3DCurves[0]->GetNaturalInterval().Evaluate(.45), sBasePoint);
        }

        smgfx_Erase();
        smgfx_SetLook(1, 2, 0, 0, 1);
        if (pBrep)
            pBrep->Draw(TRUE);
        sm_GraphicsLoop();
        smgfx_SetLook(2, 3, 0, 1, 0);
        for (ULONG ii = 0; ii < cr3DCurves.GetSize(); ii++)
        {
            if (cr3DCurves[ii])
                cr3DCurves[ii]->Draw(NULL, TRUE);
            sm_GraphicsLoop();
        }
        smgfx_SetLook(4, 5, 0, 1, 1);
        sBasePoint.Draw();
        sm_GraphicsLoop();
        smgfx_SetLook(1, 2, 0, 1, 1);
        (dSweepDist * crSweepVec).Draw(&sBasePoint);
        sm_GraphicsLoop();
        sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

    SmVector3d sOrigin(0, 0, 0);
    SmVector3d xAxis(1, 0, 0);
    SmVector3d yAxis(0, 1, 0);
    SmAxis2Placement sFrame(sOrigin, xAxis, yAxis);
    double dStartAngleDeg = 0;
    double dEndAngleDeg = 0;
    double dRadius = 0.0;
    SmStatus eStat;

    // for every curve - Extrude along vector
    for (ULONG ii = 0; ii < cr3DCurves.GetSize(); ii++)
    {
        SmBSplineCurve* pCrvToExtrude = (SmBSplineCurve*)cr3DCurves[ii];
        pExtSurface = NULL;

        // set Analytic domain
        if (pCrvToExtrude->IsAnalytic())
        {
            SmExtent1d sCrvAnalDomain = pCrvToExtrude->GetSTEPInterval();
            sAnalDomain.SetUInterval(sCrvAnalDomain);
        }
        else
        {
            SmExtent1d sCrvNurbDomain = pCrvToExtrude->GetNaturalInterval();
            sAnalDomain.SetUInterval(sCrvNurbDomain);
        }

        // If the generation curve is a line, then create an SmPlane
        if (pCrvToExtrude->IsLinear())
        {
            SmPoint3d startPt, endPt;
            pCrvToExtrude->GetEnds(startPt, endPt);
            SmVector3d sLineVector = endPt - startPt;
            SmVector2d sScale(sLineVector.Length(), dSweepDist);
            SmExtent2d sDomain(0.0, 0.0, 1.0, 1.0);

            sLineVector.Unitize();

            // SMLib requires that SmPlanes be a rectangle
            // Therefore, we check for an orthogonal sweep vector
            // Use the same criterion that is used in the SmPlane constructor:
            if (smos_Fabs(sLineVector.Dot(crSweepVec)) < SM_EFF_ZERO)
            {
                SmPlane* pPlane = new (*pContext) SmPlane(startPt, sLineVector, crSweepVec, sScale, sDomain);
                if (pPlane != NULL)
                {
                    pExtSurface = (SmSurface*)pPlane;
                }
            }
        }
        // If the generation curve is a circle, then create an SmCylinder
        else if (pCrvToExtrude->IsArc(8, dTol, sFrame, dRadius, dStartAngleDeg, dEndAngleDeg))
        {
            double dSweepAngleDeg = dEndAngleDeg - dStartAngleDeg;

            // This can be extended to create partial cylinders
            // Especially if I had access to protected constructor
            if (dSweepAngleDeg < 360.0 + dTol && dSweepAngleDeg > 360.0 - dTol)
            {
                SmVector3d zAxis = sFrame.GetZAxis();
                if (zAxis.IsParallelTo(crSweepVec, 0.5))
                {
                    // If parallel, we still must check if it is opposite
                    if (crSweepVec.Dot(zAxis) < 0.0)
                    {
                        sFrame.RotateAboutAxisAtPoint(180 * NL_PI / 180, sFrame.GetOrigin(), sFrame.GetXAxis());
                    }
                    SmCylinder* pCylinder = NULL;
                    eStat = SmCylinder::CreateCanonical(*pContext, sFrame, dRadius, pCylinder);

                    if (eStat == SM_SUCCESS && pCylinder != NULL)
                    {
                        // bound cylinder
                        SmExtent2d sExtent;
                        sExtent.SetMinMax(0, 0, 360.0, dSweepDist);
                        pCylinder->AdjustSTEPUVDomain(sExtent);

                        pExtSurface = (SmSurface*)pCylinder;
                    }
                }
            }
        }

        // Otherwise create an SmSurfOfExtrusion
        if (pExtSurface == NULL)
        {
            SmSurfOfExtrusion* pSrfExt = NULL;

            // Copy the input curve.
            SmCurve* pCrvCopy = NULL;
            pCrvToExtrude->Copy(*pContext, pCrvCopy);
            SmBSplineCurve* pBSplCopy = SM_CAST_PTR(SmBSplineCurve, pCrvCopy);
            NER(pBSplCopy);

            eStat = SmSurfOfExtrusion::CreateCanonical(*pContext, pBSplCopy, crSweepVec, pSrfExt);

            if (pSrfExt != NULL && eStat == SM_SUCCESS)
            {
                pSrfExt->AdjustSTEPUVDomain(sAnalDomain);
                pExtSurface = (SmSurface*)pSrfExt;
            }
            else
            {
                if (pSrfExt != NULL)
                {
                    delete pSrfExt;
                }
                else
                {
                    delete pCrvCopy;
                }
            }
        }

#ifdef SM_DEBUG_CODE
        if (bDebugMe)
        {
            smgfx_SetLook(2, 3, 1, 0, 0);
            if (pBrep)
                pExtSurface->Draw(TRUE);
            sm_GraphicsLoop();
            smos_WriteBuffer(_T("\nCreateLinearSweep Vector: "));
            crSweepVec.Dump();
            SM_ASSERT_VALID(pCrvToExtrude);
            SM_ASSERT_VALID(pExtSurface);
        }
#endif // SM_DEBUG_CODE

        // add new surface to brep
        SmFace* pNewFace = NULL;
        pBrep->CreateFaceFromSurface(pExtSurface, pExtSurface->GetNaturalUVDomain(), pNewFace);

    } // end iter every cr3DCurve - adding extruded faces to pBrep

#ifdef SM_DEBUG_CODE
    // draw inputs
    if (bDebugMe)
    {
        pBrep->Dump();

        SmPoint3d sBasePoint;
        if (cr3DCurves.GetSize() > 0)
        {
            cr3DCurves[0]->EvaluatePoint(cr3DCurves[0]->GetNaturalInterval().Evaluate(.45), sBasePoint);
        }

        smgfx_Erase();
        smgfx_SetLook(1, 2, 0, 0, 1);
        if (pBrep)
            pBrep->Draw(TRUE);
        sm_GraphicsLoop();
        smgfx_SetLook(2, 3, 0, 1, 0);
        for (ULONG ii = 0; ii < cr3DCurves.GetSize(); ii++)
        {
            if (cr3DCurves[ii])
                cr3DCurves[ii]->Draw(NULL, TRUE);
            sm_GraphicsLoop();
        }
        smgfx_SetLook(4, 5, 0, 1, 1);
        sBasePoint.Draw();
        sm_GraphicsLoop();
        smgfx_SetLook(1, 2, 0, 1, 1);
        (dSweepDist * crSweepVec).Draw(&sBasePoint);
        sm_GraphicsLoop();
        sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

    // build a cap face bounded by the input curves
    if (bCapEnds)
    {
        // This consumes the curves passed to it.
        sCleanInputCurves.Clear();

        // create planar start cap
        SmTArray<SmFace*> sStartFaces;
        pBrep->CreatePlanarFacesWith3DCurves(m_pRegion, cr3DCurves, dTol, sStartFaces);

        // Note: Instead of copying and translating the curves to create the second cap,
        // does it make sense to simply copy and translate the first cap?

        // build sweep transform into an SmAxis2Placement object
        SmVector3d sExtrudeVector(crSweepVec.x, crSweepVec.y, crSweepVec.z);
        sExtrudeVector.Unitize();
        sExtrudeVector = sExtrudeVector * dSweepDist;
        SmAxis2Placement sTranslate(SmVector3d(0, 0, 0), SmVector3d(1, 0, 0), SmVector3d(0, 1, 0));
        sTranslate.Translate(sExtrudeVector);
        SmTArray<SmCurve*> sEndCurves;

        // for every curve - make an array of copied and translated end curves
        for (ULONG ii = 0; ii < cr3DCurves.GetSize(); ii++)
        {
            SmCurve* crvCopy = NULL;
            cr3DCurves[ii]->Copy(*pContext, crvCopy);
            crvCopy->Transform(sTranslate);
            sEndCurves.Add(crvCopy);
        }

        // create planar end cap
        SmTArray<SmFace*> endFaces;
        pBrep->CreatePlanarFacesWith3DCurves(m_pRegion, sEndCurves, dTol, endFaces);

    } // end adding endcaps to pBrep

#ifdef SM_DEBUG_CODE
    // draw inputs
    if (bDebugMe)
    {
        pBrep->Dump();

        SmPoint3d sBasePoint;
        if (cr3DCurves.GetSize() > 0)
        {
            cr3DCurves[0]->EvaluatePoint(cr3DCurves[0]->GetNaturalInterval().Evaluate(.45), sBasePoint);
        }

        smgfx_Erase();
        smgfx_SetLook(1, 2, 0, 0, 1);
        if (pBrep)
            pBrep->Draw(TRUE);
        sm_GraphicsLoop();
        smgfx_SetLook(2, 3, 0, 1, 0);
        for (ULONG ii = 0; ii < cr3DCurves.GetSize(); ii++)
        {
            if (cr3DCurves[ii])
                cr3DCurves[ii]->Draw(NULL, TRUE);
            sm_GraphicsLoop();
        }
        smgfx_SetLook(4, 5, 0, 1, 1);
        sBasePoint.Draw();
        sm_GraphicsLoop();
        smgfx_SetLook(1, 2, 0, 1, 1);
        (dSweepDist * crSweepVec).Draw(&sBasePoint);
        sm_GraphicsLoop();
        sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

    // Stitch geometry into connected topology graph
    pBrep->StitchAndOrient();

#ifdef SM_DEBUG_CODE
    if (bDebugMe)
    {
        SM_ASSERT_VALID(pBrep);
    }
#endif // SM_DEBUG_CODE

    // all done
    pBrep->m_bEditingEnabled = FALSE;
    return SM_SUCCESS;

} // end SmPrimitiveCreation::CreateLinearSweep( curves )

/*******************************************************************//**
PURPOSE: Creates a planar face from each of the given curves, 
            and extrudes them, possibly into a solid. 

NOTES: This version is similar to protected version SmPrimitiveCreation::CreateLinearSweep
       except for the way it constructs the end caps.
       This simple version assumes that there are no nested loops and the end result
       will have a single face as caps. It will be much faster.

       If the curves are not closed loops the capping will fail.
***********************************************************************/
SmStatus SmPrimitiveCreation::CreateLinearSweepSimple
(
    SmTArray<SmCurve*> & cr3DCurves,  // in : Must be coplanar - these curves are consumed
    const SmVector3d   & crSweepVec,  // in : The sweep vector
    double               dSweepDist,  // in : Sweep = SweepDist * SweepVec
    SmBoolean            bCapEnds     // in : TRUE = build end caps
)
{
  // locals
  SmBrep          * pBrep        = m_pRegion->GetBrep();
  const SmContext * pContext     = pBrep->GetContext();
  double            dTol         = pBrep->GetTolerance();
  SmExtent2d        sAnalDomain(0.0, 0.0, 1.0, dSweepDist) ;
  SmSurface*        pExtSurface = NULL;

  SmTemporaryChangeValue< SmBoolean > sDoingBool( SM_CONST_CAST(SmContext*, pContext)->GetDoingBooleanRef(), TRUE );
  // Memory management:
  // This routine consumes (or deletes) the input curves.
  // It does two things: creates the sweep surface and face,
  // then optionally creates both end caps.
  // When creating the sweep surface, each input curve is treated
  // separately: either creates an analytic surface (plane or cylinder),
  // or a general SmSurfOfExtrusion.  The SmSurfOfExtrusion consumes the input
  // curve, but the analytic methods do not.
  // For end capping, the curves are consumed: we could use the input curves
  // directly for the start end cap, but we are required to be copied (and
  // translated) for the end end cap.
  // We could go two ways: either make a copy of each curve that gets
  // consumed in the first part and delete all input curves when done,
  // or keep track of which curves need to be deleted when done.
  // While the first option requires more copying in some cases, it is
  // much more straightforward, so we'll go that route.
  //
  //  Create sweep surface:
  //    For each curve
  //      If the curve is consumed by the surface,
  //        make a copy of it
  //
  //  If end caps:
  //    create start cap from input curves
  //    create end cap from copies
  //  else
  //    delete input curves.
  //
  // We accomplish this by setting up an SmObjsDelete to start,
  // and clearing it if we make end caps.

  SmObjsDelete< SmCurve* > sCleanInputCurves( &cr3DCurves );


#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  // draw inputs
  if(bDebugMe)
    {
      SmPoint3d sBasePoint ;
      if(cr3DCurves.GetSize() > 0)
        { cr3DCurves[0]->EvaluatePoint(cr3DCurves[0]->GetNaturalInterval().Evaluate(.45), sBasePoint) ; }

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(2,3, 0,1,0) ; for(ULONG ii=0;ii<cr3DCurves.GetSize();ii++)
                                    { if(cr3DCurves[ii]) cr3DCurves[ii]->Draw(NULL, TRUE) ; sm_GraphicsLoop() ; }
      smgfx_SetLook(4,5, 0,1,1) ; sBasePoint.Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,1) ; (dSweepDist * crSweepVec).Draw(&sBasePoint) ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE
   
  SmVector3d sOrigin( 0,0,0 );
  SmVector3d xAxis( 1,0,0 );
  SmVector3d yAxis( 0,1,0 );
  SmAxis2Placement sFrame( sOrigin, xAxis, yAxis );
  double dStartAngleDeg = 0;
  double dEndAngleDeg = 0;
  double dRadius = 0.0;
  SmStatus eStat;

  // for every curve - Extrude along vector
  for(ULONG ii=0;ii<cr3DCurves.GetSize();ii++)
    {
      SmBSplineCurve* pCrvToExtrude = (SmBSplineCurve*)cr3DCurves[ii];
      pExtSurface = NULL;
     
      // set Analytic domain
      if( pCrvToExtrude->IsAnalytic() ) 
        {
          SmExtent1d sCrvAnalDomain = pCrvToExtrude->GetSTEPInterval();
          sAnalDomain.SetUInterval( sCrvAnalDomain );
        }
      else
        {
          SmExtent1d sCrvNurbDomain = pCrvToExtrude->GetNaturalInterval();
          sAnalDomain.SetUInterval( sCrvNurbDomain );
        }

      // If the generation curve is a line, then create an SmPlane
      if( pCrvToExtrude->IsLinear( ) )
      {
          SmPoint3d startPt, endPt;
          pCrvToExtrude->GetEnds( startPt, endPt );
          SmVector3d sLineVector = endPt - startPt;
          SmVector2d sScale(sLineVector.Length(), dSweepDist);
          SmExtent2d sDomain(0.0, 0.0, 1.0, 1.0) ;

          sLineVector.Unitize();

          // SMLib requires that SmPlanes be a rectangle
          // Therefore, we check for an orthogonal sweep vector
          // Use the same criterion that is used in the SmPlane constructor:
          if (smos_Fabs( sLineVector.Dot( crSweepVec )) < SM_EFF_ZERO)
          {
            SmPlane* pPlane = new (*pContext) SmPlane(startPt, sLineVector, crSweepVec, sScale, sDomain);
            if( pPlane != NULL )
              { pExtSurface = (SmSurface*)pPlane; }
          }
      }
      // If the generation curve is a circle, then create an SmCylinder
      else if( pCrvToExtrude->IsArc( 8, dTol, sFrame, dRadius, dStartAngleDeg, dEndAngleDeg ) )
      {
          double dSweepAngleDeg = dEndAngleDeg - dStartAngleDeg;

          // This can be extended to create partial cylinders
          // Especially if I had access to protected constructor 
          if( dSweepAngleDeg < 360.0 + dTol && dSweepAngleDeg > 360.0 - dTol )
          {
              SmVector3d zAxis = sFrame.GetZAxis();
              if( zAxis.IsParallelTo( crSweepVec, 0.5 ) )
              {
                  // If parallel, we still must check if it is opposite
                  if( crSweepVec.Dot( zAxis ) < 0.0 ) { 
                    sFrame.RotateAboutAxisAtPoint( 180 * NL_PI / 180, sFrame.GetOrigin(), sFrame.GetXAxis() );
                  }
                  SmCylinder* pCylinder = NULL;
                  eStat = SmCylinder::CreateCanonical( *pContext, sFrame, dRadius, pCylinder );

                  if( eStat == SM_SUCCESS && pCylinder != NULL ) 
                  {
                      // bound cylinder
                      SmExtent2d sExtent;
                      sExtent.SetMinMax( 0, 0, 360.0, dSweepDist );  
                      pCylinder->AdjustSTEPUVDomain( sExtent );

                      pExtSurface = (SmSurface*)pCylinder;
                  }
              }
          }
      }

      // Otherwise create an SmSurfOfExtrusion
      if( pExtSurface == NULL )
      {
          SmSurfOfExtrusion* pSrfExt = NULL;

          // Copy the input curve.
          SmCurve * pCrvCopy = NULL;
          pCrvToExtrude->Copy( *pContext, pCrvCopy );
          SmBSplineCurve * pBSplCopy = SM_CAST_PTR( SmBSplineCurve, pCrvCopy );
          NER( pBSplCopy );

          eStat = SmSurfOfExtrusion::CreateCanonical( *pContext,
                                      pBSplCopy, crSweepVec, pSrfExt );
  
          if( pSrfExt != NULL  && eStat == SM_SUCCESS )
          {
              pSrfExt->AdjustSTEPUVDomain(sAnalDomain);
              pExtSurface = (SmSurface*)pSrfExt;
          }
          else
          {
              if ( pSrfExt != NULL )
                { delete pSrfExt; }
              else
                { delete pCrvCopy; }
          }
      }

#ifdef SM_DEBUG_CODE
      if(bDebugMe)
        {
          smgfx_SetLook(2,3, 1,0,0) ; if(pBrep) pExtSurface->Draw(TRUE) ; sm_GraphicsLoop() ;
          smos_WriteBuffer(_T("\nCreateLinearSweep Vector: ")) ; crSweepVec.Dump() ;
          SM_ASSERT_VALID(pCrvToExtrude) ;
          SM_ASSERT_VALID(pExtSurface) ;
        }
#endif // SM_DEBUG_CODE

      // add new surface to brep
      SmFace *pNewFace = NULL ;
      pBrep->CreateFaceFromSurface(pExtSurface, pExtSurface->GetNaturalUVDomain(), pNewFace );
    
    } // end iter every cr3DCurve - adding extruded faces to pBrep

#ifdef SM_DEBUG_CODE
  // draw inputs
  if(bDebugMe)
    {
      pBrep->Dump() ;

      SmPoint3d sBasePoint ;
      if(cr3DCurves.GetSize() > 0)
        { cr3DCurves[0]->EvaluatePoint(cr3DCurves[0]->GetNaturalInterval().Evaluate(.45), sBasePoint) ; }

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(2,3, 0,1,0) ; for(ULONG ii=0;ii<cr3DCurves.GetSize();ii++)
                                    { if(cr3DCurves[ii]) cr3DCurves[ii]->Draw(NULL, TRUE) ; sm_GraphicsLoop() ; }
      smgfx_SetLook(4,5, 0,1,1) ; sBasePoint.Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,1) ; (dSweepDist * crSweepVec).Draw(&sBasePoint) ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // build a cap face bounded by the input curves
  if( bCapEnds ) 
    {
      // This consumes the curves passed to it.
      sCleanInputCurves.Clear();

      SmFace* sStartFace = NULL;
      pBrep->CreatePlanarFaceWith3DCurves(m_pRegion, cr3DCurves, dTol, sStartFace);

      // Note: Instead of copying and translating the curves to create the second cap,
      // does it make sense to simply copy and translate the first cap?

      // build sweep transform into an SmAxis2Placement object
      SmVector3d sExtrudeVector(crSweepVec.x, crSweepVec.y, crSweepVec.z);
      sExtrudeVector.Unitize();
      sExtrudeVector = sExtrudeVector * dSweepDist;
      SmAxis2Placement sTranslate(SmVector3d(0, 0, 0), SmVector3d(1, 0, 0), SmVector3d(0, 1, 0));
      sTranslate.Translate(sExtrudeVector);

      // create planar end cap by copying and translating start cap
      SmBrep* pTmpBrep = new (*pContext) SmBrep();
      SmTArray<SmFace*> sStartFaces;
      sStartFaces.Add(sStartFace);
      pBrep->CopyFaces(sStartFaces, pTmpBrep);
      pTmpBrep->Transform(sTranslate);
      pBrep->MergeBrep(*pTmpBrep);
      delete pTmpBrep;

    } // end adding endcaps to pBrep

#ifdef SM_DEBUG_CODE
  // draw inputs
  if(bDebugMe)
    {
      pBrep->Dump() ;

      SmPoint3d sBasePoint ;
      if(cr3DCurves.GetSize() > 0)
        { cr3DCurves[0]->EvaluatePoint(cr3DCurves[0]->GetNaturalInterval().Evaluate(.45), sBasePoint) ; }

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(2,3, 0,1,0) ; for(ULONG ii=0;ii<cr3DCurves.GetSize();ii++)
                                    { if(cr3DCurves[ii]) cr3DCurves[ii]->Draw(NULL, TRUE) ; sm_GraphicsLoop() ; }
      smgfx_SetLook(4,5, 0,1,1) ; sBasePoint.Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,1) ; (dSweepDist * crSweepVec).Draw(&sBasePoint) ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // Stitch geometry into connected topology graph
  pBrep->StitchAndOrient();

#ifdef SM_DEBUG_CODE
  if(bDebugMe)
    {
      SM_ASSERT_VALID(pBrep) ;
    }
#endif // SM_DEBUG_CODE
  
  // all done 
  pBrep->m_bEditingEnabled = FALSE;
  return SM_SUCCESS;

} // end SmPrimitiveCreation::CreateLinearSweepSimple( curves )

/*******************************************************************//**
PURPOSE: Creates a planar face from the given curves, and sweeps them,
         possibly into a solid. The face may have internal loops.

NOTES:
   If the vertices/edges of the sweep happen to coincide with any
   vertices/edges of the Brep, they will be stitched together (cf SmStitch)

   The Brep is assumed to have a valid tolerance set.

   For now, the region must be the infinite region of the Brep (limitation of SmStitch).

   No curve must be degenerate.

   Curve endpoints must have exactly one match (within tolerance of Brep).

   If bCapEnds is set, repetitions need to be 1 (not implemented otherwise).

   If the Curves do not form closed loops, the capping will fail.
***********************************************************************/
SmStatus SmPrimitiveCreation::CreateLinearSweep
 (SmTArray < SmCurve*> & cr3DCurves,      // in : Must be coplanar - (consumed)
  const SmVector3d     & crSweepVec,      // in : The sweep vector
  double                 dSweepDist,      // in : Sweep = SweepDist * SweepVec
  ULONG                  nRepetitions,    // in : Number of end to end sweeps
  SmBoolean              bCapEnds,        // in : TRUE = build end caps 
  SmBoolean              bTestContinuity) // in : TRUE = split input curves at C1 discontinuties
                                          //      FALSE= use input curves as is. default:[TRUE]
{
  // empty entity maps - m_vMapToSame, m_vMapToHigher, m_vMapFromSame, and m_vMapFromLower
  InitEntityMaps();

  // See condition in usage notes:
  if (bCapEnds && nRepetitions != 1) 
    { SER(SM_ERR); }
  
  // local context, brep, region, and tolerance values
  SmBrep          * pBrep    = m_pRegion->GetBrep();
  const SmContext * pContext = pBrep->GetContext();
  double            dTol     = pBrep->GetTolerance();

  // prepare Brep for editing
  pBrep->m_bEditingEnabled = TRUE;
  SmTemporaryChangeValue< SmBoolean > sDoingBool( SM_CONST_CAST(SmContext*, pContext)->GetDoingBooleanRef(), TRUE );

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  // draw inputs
  if(bDebugMe)
    {
      SmPoint3d sBasePoint ;
      if(cr3DCurves.GetSize() > 0)
        { cr3DCurves[0]->EvaluatePoint(cr3DCurves[0]->GetNaturalInterval().Evaluate(.45), sBasePoint) ; }

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(2,3, 0,1,0) ; for(ULONG ii=0;ii<cr3DCurves.GetSize();ii++)
                                    { if(cr3DCurves[ii]) cr3DCurves[ii]->Draw(NULL, TRUE) ; sm_GraphicsLoop() ; }
      smgfx_SetLook(4,5, 0,1,1) ; sBasePoint.Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,1) ; (dSweepDist * crSweepVec).Draw(&sBasePoint) ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // when asked - split input curves with c1 discontinuities
  if (bTestContinuity) 
    {
      // look for c1 discontinuities and replace with multiple curves
      SmTArray <SmBSplineCurve *>c3DBSPLC1;

      for (ULONG ii = 0; ii < cr3DCurves.GetSize(); ii++)
        {
          // split input curves at c1 discontinuities
          ((SmBSplineCurve *)cr3DCurves[ii])->SubdivideAtDiscontinuity
                                              (*pContext,  // in : context for new object construction
                                               SM_CT_C1,   // in : curve is split at internal continuities <= to this value
                                               c3DBSPLC1); // out: newly copied curves - may be 1 curve copy when no internal discontinuities

          // when curve was split
          if ( c3DBSPLC1.GetSize() > 1 )
            {
              // delete and remove the input curve with c1 discontinuities
              delete cr3DCurves[ii]; 
              cr3DCurves.RemoveAt(ii);

              // add every split piece back into input
              for (ULONG jj = 0; jj < c3DBSPLC1.GetSize(); jj++)
                {
                  SmCurve *pCrv =(SmCurve *)(c3DBSPLC1[jj]);
                  cr3DCurves.InsertAt(ii++, pCrv);
                }
              ii--;
            } // end curve was split check
          else if ( c3DBSPLC1.GetSize() == 1 ) {
              delete c3DBSPLC1.GetAt(0);
            }
        } // end iter every input curve
    } // bTestContinuity == TRUE check

#ifdef USE_NEW_ANALYTICS
  // Use new, analytic version of this function when possible 
  if( nRepetitions == 1 ) 
    {
      return( CreateLinearSweep( cr3DCurves, crSweepVec, dSweepDist, bCapEnds ) );
    }
#endif // USE_NEW_ANALYTICS

  // Construct sweeps with multiple repetitions
  SmTArray < SmFace*> aFaces;
  SmTArray < SmEdge*> sEdges;

  // try to build a face bounded by the input curves
  if(pBrep->CreatePlanarFacesWith3DCurves( m_pRegion, 
                                           cr3DCurves, 
                                           dTol,
                                           aFaces ) != SM_SUCCESS )
    {
      
      // in which case, for every input curve - create a wire
      // which we know will fail for open loops and non-planar curves
      for ( ULONG ii = 0; ii < cr3DCurves.GetSize(); ii++ )
        {
          SmEdge  *pEdge = NULL;
          SmCurve *pCurve = cr3DCurves[ii];
          if(pCurve == NULL)
            { return SM_ERR; }

          // build and save the wire
          if(pBrep->CreateWireEdgeFromCurve(pCurve, pCurve->GetNaturalInterval(), pEdge) != SM_SUCCESS)
            { return SM_ERR; }
          sEdges.Add(pEdge);
        } // end iter every input curve building wires

      aFaces.SetSize(0);
    } // end input curves don't form coplanar closed loops branch

  else // input curves form coplanar closed loops
    {
      SmTArray < SmEdge*> sFaceEdges;

      // for every face - save the face->edgs
      for (ULONG ii = 0; ii < aFaces.GetSize(); ii++)
        {
          aFaces[ii]->GetEdges(sFaceEdges);
          sEdges.Append(sFaceEdges);
        }      
    } // end input curves do form coplanar closed loops branch
  
  // if no sweep distance - just return the brep as currently modified.
  //   if input curves form coplanar loops: Brep has trimmed plane faces
  //   else                               : Brep has wire edges
  if ( dSweepDist == 0.0 )
    { return SM_SUCCESS; }
  
  // make sure sEdges is set for the planar closed-loop case
  if ( aFaces.GetSize() != 0 )
    {
      SmFace *pBaseFace = aFaces[0];
      pBaseFace->GetEdges(sEdges); 
    }
  else // input curves are not coplanar closed-loop case
    {
      bCapEnds = FALSE;
    }

#ifdef SM_DEBUG_CODE
  // draw 
  if(bDebugMe)
    {
      SM_ASSERT_VALID(pBrep) ;
      pBrep->Dump() ;

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 0,1,1) ; for(ULONG di=0;di<cr3DCurves.GetSize();di++) 
                                   { smgfx_ChangeColor(di!=0) ; cr3DCurves[di]->Draw(NULL,TRUE) ; sm_GraphicsLoop() ; }
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE
  
  // sweep locals
  SmTranslationalSweepGeometry sTSG  ( dSweepDist * crSweepVec, dTol ) ;
  SmTopologySweep              sSweep( sTSG ) ;

  sSweep.SetTagging( m_lTagID > 0, m_lTagID ) ; 
  sSweep.SetRepetitions( nRepetitions );
  
  // when asked and it's possible - build end cap faces
  if ( bCapEnds )
    {
      sSweep.SetDoStitching( FALSE );

      SER( sSweep.DoSweep( pBrep, 
                           m_pRegion,
                          &aFaces,     // in : sweep these faces into solids
                           NULL, 
                           NULL, 
                           NULL, 
                           NULL, 
                           NULL ));   // note: increments unlocked mark value

      // Do stitch here to speed things up.
      SmSweepStitchCallback sStitchCallback( &sSweep );
      SmStitch sStitch( sStitchCallback, dTol, TRUE, FALSE );
      sStitch.m_bFastEdgeCompare        = TRUE;
      sStitch.m_bSplitEdgesWithVertices = FALSE;
      sStitch.m_bMakingManifoldSolid    = FALSE; // RCLxx TRUE commented out
      sStitch.m_bDoRegionNesting        = TRUE;
      //        sStitch.m_bSqueezeSmallEdges = TRUE;
      double dMaxVGap, dMaxEGap;
      ULONG lNumStitched, lNumLamina;
      SER( sStitch.DoStitching( pBrep, NULL, NULL, lNumStitched, lNumLamina, dMaxVGap, dMaxEGap ));

    } // end add end caps branch
  else // sweep wire edges
    {  
      SER( sSweep.DoSweep( pBrep, 
                           m_pRegion,
                           NULL, 
                          &sEdges,    // in : sweep these edges into faces
                           NULL, 
                           NULL, 
                           NULL, 
                           NULL));    // note: increments unlocked mark value


      // unrelated clean up - we didn't use the tmp faces in aFaces (if any - delete them)
      for(ULONG ii=0; ii < aFaces.GetSize(); ii++)
        {
          // FALSE: Don't delete all connected edges and vertices.
          // TRUE:  Do region nesting: combine regions/shells as appropriate.
          pBrep->DeleteFace( aFaces[ii], FALSE, TRUE );
        }
    } // end sweep wire edges branch
  
  // Another alternative for bCapEnds with repeated sweep might be 
  // the following:
  // A cap is created only for the base face and for the last face
  // Not implemented though, only if requested (quite a few lines of code)

  CopyEntityMaps( &sSweep );
  
  pBrep->m_bEditingEnabled = FALSE;
  return SM_SUCCESS;

} // end SmPrimitiveCreation::CreateLinearSweep( curves )

/*******************************************************************//**
PURPOSE: Sweeps face(s) into solid(s), from faces

NOTES: This is a protected version which makes use of SmSurfOfExtrusion
       This is only called from SmPrimitiveCreation::CreateLinearSweep
       See SmPrimitiveCreation::CreateLinearSweep for additional doc
***********************************************************************/
SmStatus SmPrimitiveCreation::CreateLinearSweep
 (SmTArray<SmFace *> & rFaces,        // in : Faces to sweep
  const SmVector3d   & crSweepVec,    // in : The sweep vector
  double               dSweepDist,    // in : The sweep distance
  SmBoolean            bCapEnds)      // in : Cap ends if true
{
  // check inputs
  if ( rFaces.GetSize() == 0 )          { SER(SM_ERR); }

  // locals
  SmBrep          * pBrep    = m_pRegion->GetBrep();
  const SmContext * pContext = pBrep->GetContext();

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  // draw inputs
  if(bDebugMe)
    {
      ULONG ii ;
      SmPoint3d sBasePoint ;
      if(rFaces.GetSize() > 0 ) 
        { rFaces[0]->CalculateAnInternalPoint(sBasePoint) ; }

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(2,3, 0,1,0) ; for(ii=0;ii<rFaces.GetSize();ii++)
                                    { if(rFaces[ii]) rFaces[ii]->DrawUV() ; sm_GraphicsLoop() ; }
      smgfx_SetLook(4,5, 0,1,1) ; sBasePoint.Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,1) ; (dSweepDist * crSweepVec).Draw(&sBasePoint) ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // locals
  ULONG ii, jj;
  SmTArray<SmCurve*> s3DCurves;
  SmTArray<SmEdge*>  sEdges;

  // for every face - build an array of copied face->edges
  for(ii=0;ii<rFaces.GetSize();ii++)
    {
      sEdges.RemoveAll();
      rFaces[ii]->GetEdges( sEdges );
      
      // for every Face->edges
      for(jj=0;jj<sEdges.GetSize();jj++)
        {
          SmCurve* crv = sEdges[jj]->GetCurve();
          SmCurve* crvCopy = NULL;
          crv->Copy( *pContext, crvCopy );

          SmExtent1d sInt = sEdges[jj]->GetInterval();
          crvCopy->Trim( sInt );
          s3DCurves.Add( crvCopy );
        }
    } // end iter every face

  // all done - pass the call along
  return( CreateLinearSweep( s3DCurves, crSweepVec, dSweepDist, bCapEnds ) );

} // end SmPrimitiveCreation::CreateLinearSweep( Faces without nRepetitions given)

/*******************************************************************//**
PURPOSE: Sweeps face(s) into solid(s), from faces

NOTES: If the vertices/edges of the sweep happen to coincide
                with any vertices/edges of the Brep, they will be 
                stitched together (cf SmStitch)

                The Brep is assumed to have a valid tolerance set.

                For now, the region must be the infinite region of the 
                Brep (limitation of SmStitch).

                If bCapEnds is set, repetitions need to be 1 (not implemented
                otherwise)        
***********************************************************************/
SmStatus SmPrimitiveCreation::CreateLinearSweep
 (SmTArray<SmFace *> & rFaces,        // in : faces to sweep
  const SmVector3d   & crSweepVec,    // in : The sweep vector
  double               dSweepDist,    // in : The sweep distance
  ULONG                nRepetitions,  // in : Number of end to end sweeps
  SmBoolean            bCapEnds)      // in : Cap ends if true
{ 
  // empty entity maps - m_vMapToSame, m_vMapToHigher, m_vMapFromSame, and m_vMapFromLower
  InitEntityMaps();

  // check inputs
  if ( bCapEnds && nRepetitions != 1 )  { SER(SM_ERR); }
  if ( rFaces.GetSize() == 0 )          { SER(SM_ERR); }

  // locals
  SmBrep * pBrep = m_pRegion->GetBrep();
  double   dTol  = pBrep->GetTolerance();
  
  // set Brep for editing  
  pBrep->m_bEditingEnabled = TRUE;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  // draw inputs
  if(bDebugMe)
    {
      ULONG ii ;
      SmPoint3d sBasePoint ;
      if(rFaces.GetSize() > 0 ) 
        { rFaces[0]->CalculateAnInternalPoint(sBasePoint) ; }

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(2,3, 0,1,0) ; for(ii=0;ii<rFaces.GetSize();ii++)
                                    { if(rFaces[ii]) rFaces[ii]->DrawUV() ; sm_GraphicsLoop() ; }
      smgfx_SetLook(4,5, 0,1,1) ; sBasePoint.Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,1) ; (dSweepDist * crSweepVec).Draw(&sBasePoint) ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

#ifdef USE_NEW_ANALYTICS
  if( nRepetitions == 1 ) 
    {
      return( CreateLinearSweep( rFaces, crSweepVec, dSweepDist, bCapEnds ) );
    }
#endif // USE_NEW_ANALYTICS

  // sweep locals
  SmTranslationalSweepGeometry sTSG  ( dSweepDist * crSweepVec, dTol );
  SmTopologySweep              sSweep( sTSG );
  
  sSweep.SetTagging( m_lTagID > 0, m_lTagID );
  sSweep.SetRepetitions( nRepetitions );

  // when adding end caps
  if ( bCapEnds )
    {
      sSweep.SetDoStitching( FALSE );
      SER( sSweep.DoSweep( pBrep, 
                           m_pRegion,
                           &rFaces, 
                           NULL,
                           NULL,NULL,NULL,NULL )); // 1 face high
                                                   // note: increments unlocked mark value
      // Do stitch here to speed things up.
      SmSweepStitchCallback sSweepStitchCallback( &sSweep );
      SmStitch sStitch( sSweepStitchCallback, dTol, TRUE, FALSE );
      sStitch.m_bFastEdgeCompare        = TRUE;
      sStitch.m_bSplitEdgesWithVertices = FALSE;
      sStitch.m_bMakingManifoldSolid    = FALSE; // RCLxx TRUE commented out
      sStitch.m_bDoRegionNesting        = TRUE;
//        sStitch.m_bSqueezeSmallEdges    = TRUE;
      double dMaxVGap, dMaxEGap;
      ULONG lNumStitched, lNumLamina;
      SER( sStitch.DoStitching( pBrep, NULL, NULL, lNumStitched, lNumLamina, dMaxVGap, dMaxEGap ));
    } // end bCapEnds == TRUE branch
  else // bCapsEnds == FALSE
    {  
      SmTArray<SmEdge*> sEdges;
      pBrep->GetEdges( sEdges );
      SER(sSweep.DoSweep(pBrep,
                         m_pRegion,
                         NULL,
                        &sEdges,
                         NULL,NULL,NULL,NULL)); // edges high
                                                // note: increments unlocked mark value
      ULONG i;
      for ( i=0; i<rFaces.GetSize(); i++ )
        {
          //
          pBrep->DeleteFace( rFaces[i], 
                             FALSE,      // FALSE: Don't delete all connected edges and vertices.           
                             TRUE );     // TRUE:  Do region nesting: combine regions/shells as appropriate.
        }
    } // end bCapsEnds == FALSE branch

  // Another alternative for bCapEnds with repeated sweep might be 
  // the following:
  // A cap is created only for the base face and for the last face
  // Not implemented though, only if requested (quite a few lines of code)

  // save side effects and restore state
  CopyEntityMaps( &sSweep );

  // all done 
  pBrep->m_bEditingEnabled = FALSE;
  return SM_SUCCESS;

} // end SmPrimitiveCreation::CreateLinearSweep( Faces with nRepetitions given)

/*******************************************************************//**
PURPOSE: Creates an analytic solid by revolving the given curve about an axis

NOTES:   Revolving a line 
             SmCylinder: If the line is parallel to the axis
             SmCone    : If the interior of the line does not intersect the axis
         Revolving an arc
             SmSphere  : If the arc is 180 deg and the endpoints rest on the axis
             SmTorus   : If the arc is 360 deg and the arc does not intersect the axis
***********************************************************************/
SmStatus sm_CreateAnalyticRotationalSweep
 (SmCurve*           & r3DCurve,        // in : Curve to revolve. Must be coplanar. Not consumed.
  const SmPoint3d    & crRotAxisBasePt, // in : Position of axis of revolution
  const SmVector3d   & crRotAxisDir,    // in : Direction of axis of revolution
  SmBrep*              pBrep )          // i/o:
{

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
    if(bDebugMe)
    {
        smgfx_Erase() ;
        smgfx_SetLook(1,2, 0,0,1) ; r3DCurve->Draw() ; sm_GraphicsLoop() ;
        smgfx_SetLook(1,2, 0,1,0) ; crRotAxisDir.Draw(&crRotAxisBasePt) ; sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE
    
    const SmContext* pContext = pBrep->GetContext();
    double dBrepTol = pBrep->GetTolerance();

    SmSurface* pAnalSrf = NULL;

    SmAxis2Placement sFrame;
    double   dRadius = 0.0;
    double   dStartAngleDeg = 0;
    double   dEndAngleDeg = 0;
    double   dStartDist, dEndDist;
    SmFace*  pFace = NULL;
    SmStatus eStat = SM_ERR;

    SmPoint3d sStartPt, sEndPt;
    r3DCurve->GetEnds( sStartPt, sEndPt );

    smgu_LinePointDistance( crRotAxisBasePt, crRotAxisDir, sStartPt, dStartDist, NULL );
    smgu_LinePointDistance( crRotAxisBasePt, crRotAxisDir, sEndPt,   dEndDist,   NULL );

    // Revolving a line can be a cylinder or a cone
    if( r3DCurve->IsLinear( )  )
    {
        SmVector3d sLineVector = sEndPt - sStartPt;
        double dLineLength = sStartPt.DistanceBetween( sEndPt );
        if ( dLineLength < dBrepTol )
          { SER( SM_ERR_INVALID_INPUT ); }
        sLineVector /= dLineLength;

        // Get an appropriate angle tolerance.  Both ends of line should be
        // within 3d tol of parallel or perpendicular.
        double dLength = smos_4Max( dStartDist, dEndDist, dLineLength, SM_EFF_ZERO_SQRT );
        double dAngleTolDeg = SM_RAD2DEG( dBrepTol / dLength );

        // SmCylinder: Line must parallel to axis
        //             Line must not be coincident with axis
        if ( sLineVector.IsParallelTo( crRotAxisDir, dAngleTolDeg ) )
        {
            if ( dStartDist <= dBrepTol )
              { return SM_ERR; } // line on axis

            // Given curve is a line parallel to the axis and not coincident with it.

            // X axis: from the axis towards the line
            // Y axis: for right-handed rotation
            // Note these vectors can't be zero because sStartPt is off the axis.
            SmVector3d sDiffVec( sStartPt - crRotAxisBasePt );
            SmVector3d sYAxis  ( sLineVector * sDiffVec );
            SER( sYAxis.Unitize() );
            SmVector3d sXAxis  ( sYAxis * sLineVector );

            // Find the base point for the cylinder.
            // Drop the Start point to the axis.
            double dT;
            smgu_LineClosestPoint( crRotAxisBasePt, crRotAxisDir, sStartPt, dT );
            SmPoint3d sBasePoint = crRotAxisBasePt + dT * crRotAxisDir;

            sFrame.SetCanonical( sBasePoint, sXAxis, sYAxis );

            SmCylinder* pCylinder = NULL;
            eStat = SmCylinder::CreateCanonical( *pContext, sFrame, dStartDist, pCylinder ); 

            if( eStat == SM_SUCCESS ) 
            {
                double dHeight = sStartPt.DistanceBetween( sEndPt );
                SmExtent2d sExtent;
                sExtent.SetMinMax( 0, 0, 360.0, dHeight );  
                pCylinder->AdjustSTEPUVDomain( sExtent );

                pAnalSrf = pCylinder;
            }
        } // end if we can create a cylinder

        // SmPlane: To avoid revolving a disk we will create a circular plane in this case
        else if( sLineVector.IsPerpendicularTo( crRotAxisDir, dAngleTolDeg )  )
        {
            // Build circle(s).
            SmCircle* pCircle = NULL;
            SmTArray <SmCurve*> sCurves;

            // Find the center point for circles.
            // Both Start and End points should drop to the same point.
            double dT;
            smgu_LineClosestPoint( crRotAxisBasePt, crRotAxisDir, sStartPt, dT );
            SmPoint3d sBasePoint = crRotAxisBasePt + dT * crRotAxisDir;

            double dEndDistance   = 0.0;
            smgu_LinePointDistance( crRotAxisBasePt, crRotAxisDir, sEndPt, dEndDistance );

            // Reference frame: Make the vector point away from the axis,
            // and make the rotation CCW (right-hand rule).
            // (Just for consistency; probably doesn't actually make a difference.)
            // sLineVector is the x-axis.
            if ( dStartDist > dEndDistance)
              { sLineVector *= -1.0; }
            SmVector3d sYAxis = crRotAxisDir * sLineVector;
            sFrame.SetCanonical( sBasePoint, sLineVector, sYAxis );

            SmScaledZero dScaledZero = SmTol::GetScaledZero(sStartPt, sEndPt) ;

            if ( dStartDist > dScaledZero )
              {
                SmCircle::CreateCanonical( *pContext, sFrame, dStartDist, pCircle );
                sCurves.Add( pCircle );
              }
            if (dEndDistance > dScaledZero )
              {
                SmCircle::CreateCanonical( *pContext, sFrame, dEndDistance, pCircle );
                sCurves.Add( pCircle );
              }

            // create circular plane
            double dTol = pBrep->GetTolerance(); // Can't be too loose.  [B447]
            eStat = pBrep->CreatePlanarFaceWith3DCurves( pBrep->GetInfiniteRegion(), // in : region to contain newFace - not checked
                                                                                     //      NULL for infinite region.
                                                         sCurves,                    // in : array of planar curves to bound newFace - curves used by NewEdges
                                                         dTol,                       // in : 3d Distance for planarity checks
                                                         pFace );                    // out: new face

            // Since we have already added the face to the brep 
            pAnalSrf = NULL;

        } // end if we can create a plane.

        // SmCone: Line must not intersect axis at any points other than the endpoints
        else 
        {
            // Determine radius by dropping start pt onto axis 
            double dBottomRadius = 0.0;
            double dBottomParam = 0.0;
            smgu_LinePointDistance( crRotAxisBasePt, crRotAxisDir, sStartPt, dBottomRadius, &dBottomParam );

            double dTopRadius = 0.0;
            double dTopParam = 0.0;
            smgu_LinePointDistance( crRotAxisBasePt, crRotAxisDir, sEndPt, dTopRadius, &dTopParam );

            SmLine sAxisLine( crRotAxisBasePt, crRotAxisDir, 3, TRUE, pContext, FALSE );

            SmSolutionArray sSolutions;
            sAxisLine.GlobalCurveIntersect( sAxisLine.GetNaturalInterval(),
                                            *r3DCurve, r3DCurve->GetNaturalInterval(), dBrepTol,
                                            sSolutions );

            if( sSolutions.GetSize() > 0 ) 
            {
                SmExtent1d sExtent = r3DCurve->GetNaturalInterval();
                double dParamTol   = SM_EFF_ZERO_SQRT/100.0 * (1.0 + sExtent.GetLength()) ;

                // Two lines can only have one solution (one point or all points)
                if (sSolutions[0].m_eSolutionType == SM_ST_SINGLE_VALUE) 
                {
                    // Get parameter of intersection on second curve
                    double dT = sSolutions[0].m_vStart[1];

                    if( !( fabs( sExtent.GetMin() - dT ) < dParamTol || fabs( sExtent.GetMax() - dT ) < dParamTol  ) )
                        return( SM_ERR );
                }
                else if (sSolutions[0].m_eSolutionType == SM_ST_RANGE_OF_VALUES)
                  { return( SM_ERR ); } // input line on axis.
            }

            SmPoint3d sBottomPt;
            sAxisLine.EvaluatePoint( dBottomParam, sBottomPt );

            SmPoint3d sTopPt;
            sAxisLine.EvaluatePoint( dTopParam, sTopPt );

            // The height is the distance between drop points.
            double dHeight = sBottomPt.DistanceBetween( sTopPt );

            // X axis: from the axis towards the line
            // Y axis: for right-handed rotation
            // Note these vectors can't be zero because sStartPt is off the axis.
            double dDot = crRotAxisDir.Dot( sLineVector );
            SmVector3d sTmpVector = dDot < 0.0 ? crRotAxisDir * -1 : crRotAxisDir;

            SmVector3d sDiffVec( sStartPt - crRotAxisBasePt );
            SmVector3d sYAxis  ( sTmpVector * sDiffVec );
            SER( sYAxis.Unitize() );
            SmVector3d sXAxis  ( sYAxis * sTmpVector );

            sFrame.SetCanonical( sBottomPt, sXAxis, sYAxis );

            SmCone* pCone = NULL;
            eStat = SmCone::CreateCanonical( *pContext, sFrame, dBottomRadius, dTopRadius, dHeight, pCone ); 
            if( eStat == SM_SUCCESS )
            {
                pAnalSrf = pCone;
            }
        } // end else creating a cone

    } // end if curve is linear

    // Revolving an arc can be a sphere or a torus
    // Tol should be a function of size
    else if( ((SmBSplineCurve*)r3DCurve)->IsArc( 8, dBrepTol, sFrame, dRadius, dStartAngleDeg, dEndAngleDeg ) )
    {
        double dSweepAngleDeg = dEndAngleDeg - dStartAngleDeg;

        // SmSphere: Curve must be a half circle
        //           Curve endpoints must be on axis
        if( dSweepAngleDeg < 180.0 + dBrepTol && dSweepAngleDeg > 180.0 - dBrepTol )
        {
            double dDist = 0.0;
            smgu_LinePointDistance( crRotAxisBasePt, crRotAxisDir, sStartPt, dDist, NULL );
            if( dDist > dBrepTol )
                return( SM_ERR );

            smgu_LinePointDistance( crRotAxisBasePt, crRotAxisDir, sEndPt, dDist, NULL );
            if( dDist > dBrepTol )
                return( SM_ERR );
               
            SmSphere* pSphere = NULL;
            eStat = SmSphere::CreateCanonical( *pContext, sFrame, dRadius, pSphere );
            if( eStat == SM_SUCCESS ) 
            {
                pAnalSrf = pSphere;
            }
        }
        // SmTorus: curve must be a full circle
        //          curve must not intersect axis
        else if( dSweepAngleDeg < 360.0 + dBrepTol && dSweepAngleDeg > 360.0 - dBrepTol )
        {
            SmLine sAxisLine( crRotAxisBasePt, crRotAxisDir, 3, TRUE, pContext, FALSE );

            SmSolutionArray sSolutions;
            sAxisLine.GlobalCurveIntersect( sAxisLine.GetNaturalInterval(), *r3DCurve, r3DCurve->GetNaturalInterval(), dBrepTol, sSolutions );
            if( sSolutions.GetSize() > 0 ) 
                return( SM_ERR );

            // The Minor radius is the radius of the circle
            // The major radius is the distance from the center of the circle to the perpedicular to the axis
            double dMajorRadius = 0.0;
            double dParamOnAxis = 0.0;
            smgu_LinePointDistance( crRotAxisBasePt, crRotAxisDir, sFrame.GetOriginRef(), dMajorRadius, &dParamOnAxis );

            // sPntOnAxis is the point where the center of the circle drops to the axis
            SmPoint3d sPntOnAxis = crRotAxisBasePt + dParamOnAxis * crRotAxisDir;

            SmTArray<SmCurve*> sInputCurves;
            sInputCurves.Add( r3DCurve );

            SmVector3d sCrvNormal;
            SmCurve::ComputeCrvsNormal( sInputCurves, sCrvNormal );

            // The frame for the torus must be based at the base of the axis
            // And oriented 
            SmVector3d xAxis = sCrvNormal;
            //if( crRotAxisDir.IsParallelTo( xAxis ) )
            //    xAxis.Set( 0,1,0 );

            SmVector3d yAxis = xAxis * crRotAxisDir;
            sFrame.SetCanonical( sPntOnAxis, xAxis, yAxis );

#ifdef SM_DEBUG_CODE
    if(bDebugMe)
    {
        smgfx_SetLook( 4,4, 1,0,0 ); xAxis.Draw( &sPntOnAxis ); sm_GraphicsLoop();
        smgfx_SetLook( 4,4, 1,1,0 ); yAxis.Draw( &sPntOnAxis ); sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

            SmTorus* pTorus = NULL;
            eStat = SmTorus::CreateCanonical( *pContext, sFrame, dMajorRadius, dRadius, pTorus );
            if( eStat == SM_SUCCESS )
            {
                pAnalSrf = pTorus;
            }
        }
    } // end else if curve is a circle

    if( eStat == SM_SUCCESS && pAnalSrf != NULL )
    {
        eStat = pBrep->CreateFaceFromSurface( pAnalSrf, pAnalSrf->GetNaturalUVDomain(), pFace );
    }

    if( eStat != SM_SUCCESS || pFace == NULL )
      { return SM_ERR; }

#ifdef SM_DEBUG_CODE
    if(bDebugMe)
    {
        if ( FALSE ) {
            smgfx_Erase() ;
            smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
        }
        smgfx_SetLook(1,2, 0,0,0); pFace->Draw( SM_DM_CROSSHATCH,2,2 ); sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

    return SM_SUCCESS;

} // end sm_CreateAnalyticRotationalSweep

/*******************************************************************//**
PURPOSE: Creates a planar face from the given curves, and sweeps them,
         possibly into a solid.

NOTES:
   The face may have internal loops.

   If the vertices/edges of the sweep happen to coincide with any
   vertices/edges of the Brep, they will be stitched together (cf SmStitch)

   The Brep is assumed to have a valid tolerance set.

   For now, the region must be the infinite region of the Brep (limitation of SmStitch).

   No curve must be degenerate.

   Curve endpoints must have exactly one match (within tolerance of Brep).

   Input curves are consumed. Sometimes the curves will be deleted in this function.
   Other times the curves will be deleted when the result is deleted.

   Note: bEndcaps here applies to a closed curve being rotated.
   For example: A circle rotated 180 degrees can have 2 closed endcaps
                A line rotated 360 will NOT be a closed cylinder,
                     since endcaps are not applied to the curve-ends.
                If bCapEnds is set, repetitions need to be 1
                    (not implemented otherwise)
***********************************************************************/
SmStatus SmPrimitiveCreation::CreateRotationalSweep
 (SmTArray<SmCurve*> & r3DCurves,       // in : Curves to revolve. Must be coplanar. They are consumed
  const SmPoint3d    & crRotAxisBasePt, // in : Position of axis of revolution
  const SmVector3d   & crRotAxisDir )   // in : Direction of axis of revolution
{
  // empty entity maps - m_vMapToSame, m_vMapToHigher, m_vMapFromSame, and m_vMapFromLower
  InitEntityMaps();

  // locals and inits
  ULONG ii;
  SmBrep          * pBrep         = m_pRegion->GetBrep();
  const SmContext * pContext      = pBrep->GetContext();
  SmBSplineCurve  * pCrvToRevolve = NULL;
  SmFace          * pFace         = NULL;
  SmStatus          eStat         = SM_SUCCESS;

  // prepare Brep for changes
  pBrep->m_bEditingEnabled = TRUE;

#ifdef SM_DEBUG_CODE
ULONG di ;
SmBoolean bDebugMe = FALSE ;
if(bDebugMe)
  {
    if(pBrep) pBrep->Dump() ;

    smgfx_Erase() ;
    smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
    smgfx_SetLook(2,3, 1,0,0) ; for(di=0;di<r3DCurves.GetSize();di++)
                                  { smgfx_ChangeColor(di != 0) ; if(r3DCurves[di]) r3DCurves[di]->Draw() ; sm_GraphicsLoop() ; }
    smgfx_SetLook(5,6, 0,1,0) ; crRotAxisBasePt.Draw() ; sm_GraphicsLoop() ;
    smgfx_SetLook(1,2, 0,1,0) ; (10.0 * crRotAxisDir).Draw(&crRotAxisBasePt) ; sm_GraphicsLoop() ;
    sm_GraphicsLoop() ;
  }
#endif // SM_DEBUG_CODE

  // If only one curve is input, we will try to catch cases where analytic surfaces can be built
  // If this fails then we have another chance below
  if( r3DCurves.GetSize() == 1 )
    {
      if( sm_CreateAnalyticRotationalSweep( r3DCurves[0], crRotAxisBasePt, crRotAxisDir, pBrep ) == SM_SUCCESS )
        {
          // This does not consume the curve.
          delete r3DCurves[0]; r3DCurves[0] = NULL;

          return( SM_SUCCESS );
        }
    }

  // for every curve
  for(ii=0;ii<r3DCurves.GetSize();ii++) 
    {
      pCrvToRevolve = (SmBSplineCurve*)r3DCurves[ii];
      pFace         = NULL;

#ifdef SM_DEBUG_CODE
      if(bDebugMe)
        {
          smgfx_Erase() ;
          smgfx_SetLook(1,2, 0,0,1) ; if(pCrvToRevolve) pCrvToRevolve->Draw() ; sm_GraphicsLoop() ;
          smgfx_SetLook(5,6, 0,1,0) ; crRotAxisBasePt.Draw() ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 0,1,0) ; (10.0 * crRotAxisDir).Draw(&crRotAxisBasePt) ; sm_GraphicsLoop() ;
          sm_GraphicsLoop() ;
        }
#endif // SM_DEBUG_CODE

      // locals
      double dLengthOfCrv = 0.0;
      double dAccuracyTol = 0.001;    // Is there a better value for this?
      pCrvToRevolve->Length( pCrvToRevolve->GetNaturalInterval(), dAccuracyTol, dLengthOfCrv );
      
      // degenerate curves are considered bad 
      // closed curves should be ok
      if ( dLengthOfCrv < SM_EFF_ZERO ) { 
          SER( SM_ERR_INVALID_INPUT ); 
      }

      SmPoint3d  sStartPt, sEndPt;
      pCrvToRevolve->GetEnds( sStartPt, sEndPt );

      double dStartDist = 0.0;
      smgu_LinePointDistance( crRotAxisBasePt, crRotAxisDir, sStartPt, dStartDist );

      double dEndDist   = 0.0;
      smgu_LinePointDistance( crRotAxisBasePt, crRotAxisDir, sEndPt, dEndDist );

      // Get an appropriate angle tolerance.  Both ends of line should be
      // within 3d tol of parallel or perpendicular.
      double dLength = smos_4Max( dStartDist, dEndDist, dLengthOfCrv, SM_EFF_ZERO_SQRT );
      double dAngleTolDeg = SM_RAD2DEG( pBrep->GetTolerance() / dLength );       // GWC: converting a linear into a rotational tolerance here - not good

      // If the curve is a line AND the line is perpendicular to the axis
      // then create a circular plane instead of a surface of revolution.
      if ( pCrvToRevolve->IsLinear() ) {
          SmVector3d sLineVector = sEndPt - sStartPt;
          double     dVecLength  = sLineVector.Length();
          sLineVector /= dVecLength;

          if( sLineVector.IsPerpendicularTo( crRotAxisDir, dAngleTolDeg ) ) {
          // Build circle(s).
          SmAxis2Placement     sFrame;
          SmCircle           * pCircle = NULL;
          SmTArray <SmCurve*>  sCurves;
          double               dParam ;
          SmScaledZero         dScaledZero =  SmTol::GetScaledZero(sStartPt, sEndPt) ;

          // Find the center point for circles.
          // Both Start and End points should drop to the same point.
          smgu_LineClosestPoint( crRotAxisBasePt, crRotAxisDir, sStartPt, dParam );
          SmPoint3d sBasePoint = crRotAxisBasePt + dParam * crRotAxisDir;

          // Reference frame: Make the vector point away from the axis,
          //                  and make the rotation CCW (right-hand rule).
          //                  (Just for consistency; probably doesn't actually make a difference.)
          //                  sLineVector is the x-axis.
          if ( dStartDist > dEndDist ) { sLineVector *= -1.0; }
          SmVector3d sYAxis = crRotAxisDir * sLineVector;
          sFrame.SetCanonical( sBasePoint, sLineVector, sYAxis );

          // start radius circle
          if ( dStartDist > dScaledZero )
            {
              SmCircle::CreateCanonical( *pContext, sFrame, dStartDist, pCircle );
              sCurves.Add( pCircle );
            }

          // end radius circle
          if ( dEndDist > dScaledZero )
            {
              SmCircle::CreateCanonical( *pContext, sFrame, dEndDist, pCircle );
              sCurves.Add( pCircle );
            }

          // create circular plane
          double dTol = pBrep->GetTolerance(); // Can't be too loose.  [B447]
          eStat = pBrep->CreatePlanarFaceWith3DCurves( pBrep->GetInfiniteRegion(), // in : region to contain newFace - not checked
                                                                                   //      NULL for infinite region.
                                                       sCurves,                    // in : array of planar curves to bound newFace - curves used by NewEdges
                                                       dTol,                       // in : 3d Distance for planarity checks
                                                       pFace );                    // out: new face
          SM_ASSERT_BREAK(pFace != NULL) ;

          // We must consume the curves - GWC: if pFace == NULL - get into problems
          delete pCrvToRevolve;  pCrvToRevolve = NULL;
          r3DCurves[ii] = NULL ;
          }

        } // end if we can create a plane.

      // If we didn't successfully create a Plane face, create a Rev Surf.
      if ( pFace == NULL && pCrvToRevolve != NULL )
        {
          // SmSurfOfRevolution creator consumes the curve if successful.
          // Otherwise we destroy the curve here.
          SmSurfOfRevolution * pRevSrf = NULL;
          eStat = SmSurfOfRevolution::CreateCanonical( *pContext,
                                                        pCrvToRevolve, 
                                                        crRotAxisBasePt, 
                                                        crRotAxisDir, 
                                                        pRevSrf );

          if ( pRevSrf != NULL && eStat == SM_SUCCESS )
            {
              eStat = pBrep->CreateFaceFromSurface( (SmSurface*)pRevSrf, pRevSrf->GetNaturalUVDomain(), pFace );
            }
          if ( pFace == NULL || eStat != SM_SUCCESS )
            {
              // Input curve must be consumed.
              delete pCrvToRevolve; 
              pCrvToRevolve = NULL; 
              r3DCurves[ii] = NULL; // Don't let the caller use it.
            }
        } // end if creating a Rev Surf

#ifdef SM_DEBUG_CODE
      if(bDebugMe) // draw pBrep (blue) and just created face (crosshatched)
        { 
          pBrep->Dump();  // AssertValid?  There will be topology coincidences here.

          if ( FALSE ) 
            { smgfx_Erase();
              smgfx_SetLook(1,2, 0,0,1); pBrep->Draw(); sm_GraphicsLoop();
            }

          smgfx_SetLook(1,2, 1,0,0); if(pFace && eStat==SM_SUCCESS) pFace->Draw(SM_DM_CROSSHATCH,2,2); sm_GraphicsLoop(); 
          sm_GraphicsLoop() ;
        }
#endif // SM_DEBUG_CODE

  } // end for each curve.

  pBrep->StitchAndOrient();

  pBrep->m_bEditingEnabled = FALSE;

#ifdef SM_DEBUG_CODE
  if(bDebugMe) 
    {
      SM_ASSERT_VALID( pBrep );
      smgfx_Erase();
      smgfx_SetLook(1,2, 0,0,1); pBrep->Draw(TRUE); sm_GraphicsLoop();
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  return SM_SUCCESS;

} // end SmPrimitiveCreation::CreateRotationalSweep

/*******************************************************************//**
PURPOSE: Creates a planar face from the given curves, and sweeps them,
            possibly into a solid. 

NOTES:
   The face may have internal loops.

   If the vertices/edges of the sweep happen to coincide with any vertices/edges
   of the Brep, they will be stitched together (cf SmStitch)

   The Brep is assumed to have a valid tolerance set.

   For now, the region must be the infinite region of the Brep
   (limitation of SmStitch).
                
   No curve must be degenerate.
                
   Curve endpoints must have exactly one match (within tolerance of Brep).

   Note: bEndcaps here applies to a closed curve being rotated.
     For example: A circle rotated 180 degrees can have two closed endcaps.
     A line rotated 360 will NOT be a closed cylinder, since endcaps
       are not applied to the curve-ends.
     If bCapEnds is set, repetitions need to be 1 (not implemented otherwise)
***********************************************************************/
SmStatus SmPrimitiveCreation::CreateRotationalSweep
 (SmTArray<SmCurve*> & r3DCurves,       // in : Curves to revolve. Must be coplanar. They are consumed
  const SmPoint3d    & crRotAxisBasePt, // in : Position of axis of revolution
  const SmVector3d   & crRotAxisDir,    // in : Direction of axis of revolution
  double               dRotAngleDeg,    // in : degrees 0->360
  ULONG                nRepetitions,    // in : Number of end to end revolutions
  SmBoolean            bCapEnds,        // in : Cap ends if true
  SmBoolean            bTestContinuity) // in : Test for tangent continuity if true
{
  // empty entity maps - m_vMapToSame, m_vMapToHigher, m_vMapFromSame, and m_vMapFromLower
  InitEntityMaps();

  // locals
  SmBrep          * pBrep    = m_pRegion->GetBrep();
  const SmContext * pContext = pBrep->GetContext();
  SmTemporaryChangeValue <SmBoolean> sTempChangeDB( ((SmContext*)pContext)->GetDoingBooleanRef(), TRUE );

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  // draw inputs
  if(bDebugMe)
    {
      ULONG ii ;

      if(pBrep) pBrep->Dump() ;

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      for(ii=0;ii<r3DCurves.GetSize();ii++)
        { double dParam = (double)ii/((double)(r3DCurves.GetSize())-1.0) ;
          smgfx_SetLook(2+ii,3, 1.0-dParam,0,dParam) ; 
          if(r3DCurves[ii]) { SM_DUMP_AND_ASSERT_VALID(r3DCurves[ii]) ; r3DCurves[ii]->Draw() ; sm_GraphicsLoop() ; }
        }
      smgfx_SetLook(5,6, 0,1,0) ; crRotAxisBasePt.Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,0) ; (10.0 * crRotAxisDir).Draw(&crRotAxisBasePt) ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  //
  if (bTestContinuity) 
    {
      // look for c1 discontinuities and replace with multiple curves
      SmTArray <SmBSplineCurve *>c3DBSPLC1;

      for (ULONG i = 0; i < r3DCurves.GetSize(); i++) 
        {
          ((SmBSplineCurve *)r3DCurves[i])->SubdivideAtDiscontinuity
                                             (*pContext,   // in : context for new object construction
                                              SM_CT_C1,    // in : curve is split at internal continuities <= to this value
                                              c3DBSPLC1);  // out: newly copied curves - may be 1 curve copy when no internal discontinuities
          if (c3DBSPLC1.GetSize() > 1) 
            {
              delete r3DCurves[i]; // delete this curve, which is now replaced
              r3DCurves.RemoveAt(i);
              for (ULONG j = 0; j < c3DBSPLC1.GetSize(); j++) 
                {
                  SmCurve *pCrv =(SmCurve *)(c3DBSPLC1[j]);
                  r3DCurves.InsertAt(i++, pCrv);
                }
              i--;
            }
          else if ( c3DBSPLC1.GetSize() == 1 ) {
              delete c3DBSPLC1.GetAt(0);
            }
        }
    }

#ifdef USE_NEW_ANALYTICS
    // Don't add end caps if full 360 degree rotation.
    if (dRotAngleDeg    >= 359.99999) 
    {
      return( CreateRotationalSweep( r3DCurves, crRotAxisBasePt, crRotAxisDir ) );
    }
#endif // USE_NEW_ANALYTICS

  // Construct partial surface of revolution

  // See condition in usage notes:
  if (bCapEnds    && nRepetitions    != 1)  
    { SER(SM_ERR); }

  double dTol = pBrep->GetTolerance();
  pBrep->m_bEditingEnabled = TRUE;
    
  SmTArray<SmFace*> aFaces;
  SmTArray<SmEdge*> sEdges;

  // This consumes the input curves (if successful).
  if (pBrep->CreatePlanarFacesWith3DCurves(m_pRegion, r3DCurves, dTol, aFaces) != SM_SUCCESS) 
    {
      for (ULONG i=0; i<r3DCurves.GetSize(); i++) 
        {
          // This also consumes the input curve (if successful).
          SmCurve *pCurve = r3DCurves   [i];
          SmEdge *pEdge = NULL;
          if (pBrep->CreateWireEdgeFromCurve(pCurve,pCurve->GetNaturalInterval(),pEdge) != SM_SUCCESS) 
            {
              return SM_ERR;
            }
          sEdges.Add(pEdge);
        }
      aFaces.SetSize(0);
    }

  SmFace *pBaseFace = NULL;
  if (aFaces.GetSize() != 0) 
    {
      pBaseFace = aFaces[0];
      SmTArray<SmEdge*> sFaceEdges;
      for (ULONG i=0; i<aFaces.GetSize(); i++) 
        {
          aFaces[i]->GetEdges(sFaceEdges);
          sEdges.Append(sFaceEdges);
        }
    }

  // Create a line that represents the axis and classify it 
  // relative to the face.
  if (pBaseFace) 
    {
      SmExtent3d sFaceBox;
      SER(pBaseFace->CalculateBoundingBox(sFaceBox));
      SmVector3d sSphCent;
      double dSphRad;
      sFaceBox.ComputeSphereBound(sSphCent,dSphRad);
      
      double dParam;
      SmVector3d sLineVec = crRotAxisDir   ;
      SER(sLineVec.Unitize());
      SER(smgu_LineClosestPoint(crRotAxisBasePt   ,sLineVec,sSphCent,dParam));
      SmPoint3d sPointOnAxis = crRotAxisBasePt    + dParam * sLineVec;
      SmPoint3d sStartPoint = sPointOnAxis - 2.0 * dSphRad * sLineVec;
      SmPoint3d sEndPoint = sPointOnAxis + 2.0 * dSphRad * sLineVec;
      
      SmBSplineCurve *pAxisCurve = NULL ;
      const SmContext *cpContext = pBrep->GetContext();
      SER(SmBSplineCurve::CreateLineSegment(*cpContext,3,sStartPoint,sEndPoint,pAxisCurve)); NER(pAxisCurve);
      SmObjDelete sCleanAxis(pAxisCurve);
      
      SmCurveClassification sCurveClass(pAxisCurve,pAxisCurve->GetNaturalInterval(),NULL,pBaseFace->GetTolerance());
      SER(pBaseFace->CurveOnClassify(TRUE,sCurveClass));
      
      // Check to see if any of intervals is in the face
      for (ULONG i=0; i<sCurveClass.GetSize(); i++) 
        {
          SmCurveInterval & rIvl = sCurveClass[i];
          if (rIvl.m_vMid.GetPointClass() == SM_PC_FACE) 
            {
              // Failure - delete the face we created in the brep
              const SmContext *pBrepContext = pBrep->GetContext();
              SmTopologyIntersector sTI(*pBrepContext );
              SmTArray<SmFace*> sFaces;
              sFaces.Add(pBaseFace);
              SER(sTI.DeleteFaces(sFaces));
              return SM_ERR_AXIS_INSIDE_FACE;
            }
        }
    }
  else 
    {
      bCapEnds    = FALSE;
    }

  SmRotationalSweepGeometry sTSG(crRotAxisBasePt, crRotAxisDir, dRotAngleDeg, dTol); 

  SmTopologySweep sSweep(sTSG);
   if (m_lTagID > 0) 
     {
      sSweep.SetTagging(TRUE,m_lTagID);
    }

  if ( bCapEnds ) 
    {
      sSweep.SetRepetitions(nRepetitions   );
      SER(sSweep.DoSweep(pBrep, m_pRegion, &aFaces,  // note: increments unlocked mark value
                         NULL,NULL,NULL,NULL,NULL)); // 1 face high
    }
  else 
    {
      sSweep.SetRepetitions(nRepetitions   );
      sSweep.SetDoStitching(FALSE);
      SER(sSweep.DoSweep(pBrep, m_pRegion, NULL,&sEdges,    // note: increments unlocked mark value
                         NULL,NULL,NULL,NULL));             // edges high
      const SmContext *pBrepContext = pBrep->GetContext();
      SmTopologyIntersector sTI(*pBrepContext );
      sTI.DeleteFaces(aFaces);

      // Now that we have deleted the interior face - stitch 
      // and do region determination.
      SmSweepStitchCallback sCallBack(&sSweep);
      SmStitch sStitch(sCallBack, pBrep->GetTolerance());
      sStitch.m_bMakingManifoldSolid = FALSE; 

      ULONG lStitchedEdges, lLaminaEdges;
      double dMaxVertexGap, dMaxEdgeGap;

      SER(sStitch.DoStitching(pBrep,
          NULL,NULL,lStitchedEdges,lLaminaEdges,
          dMaxVertexGap,dMaxEdgeGap));
    }

  // Another alternative for bCapEnds with repeated sweep might be the following:
  // A cap is created only for the start of the base face and for the end of the last face.

  CopyEntityMaps( &sSweep );

  pBrep->m_bEditingEnabled = FALSE;
  return SM_SUCCESS;
} // end SmPrimitiveCreation::CreateRotationalSweep

/*******************************************************************//**
PURPOSE: Create a Swung surface primitive.

NOTES: 
   Does not consume the input curves.
***********************************************************************/
SmStatus SmPrimitiveCreation::CreateSwungPrimitive
 (SmTArray<SmCurve*> & rXYCurvesPath,
  SmTArray<SmCurve*> & rXZCurvesProfile,
  double dScale,
  double bAppoxTol3d )
{
  // empty entity maps - m_vMapToSame, m_vMapToHigher, m_vMapFromSame, and m_vMapFromLower
  InitEntityMaps();

  double dCrvScale=0.0;
  SmTArray<SmFace*> sFaces;
  SmRegion *pReg = m_pRegion;
  const SmContext * cpContext = pReg->GetContext();
  for (ULONG i=0; i<rXYCurvesPath.GetSize(); i++) {
      SmBSplineCurve *pXYCurve = SM_CAST_PTR(SmBSplineCurve,rXYCurvesPath[i]);
      if (!pXYCurve) continue;
      for (ULONG j=0; j<rXZCurvesProfile.GetSize(); j++) {
          SmBSplineCurve *pXZCurve = SM_CAST_PTR(SmBSplineCurve,rXZCurvesProfile[j]);
          if (!pXZCurve) continue;
          if (i==0 && j==0) {
              SmPoint3d sXYPnt, sXZPnt;
              SER(pXZCurve->EvaluatePoint(pXZCurve->GetNaturalInterval().GetMin(),sXZPnt));
              double dPlaneD = 0;
              SmVector3d sPlaneVec(0.0,1.0,0.0);
              dPlaneD = -sXZPnt.Dot(sPlaneVec);
              SmSolutionArray sSolutions;
              SER(pXYCurve->GlobalPropertyAnalysis(pXZCurve->GetNaturalInterval(),
                  SM_CP_PLANE_INTERSECTION,&dPlaneD,&sPlaneVec,bAppoxTol3d,
                  sSolutions));
              if (sSolutions.GetSize() > 0) {
                  SmSolution &rSol = sSolutions[0];
                  SER(pXYCurve->EvaluatePoint(rSol.m_vStart[0],sXZPnt));
              }
              dCrvScale = smos_Fabs(1.0/(sXZPnt.x));
          }
          SmBSplineSurface *pNewSurface = NULL ;

          // This does not consume the input curves:
          SER(SmBSplineSurface::CreateSwungSurface(*cpContext,pXZCurve,
              pXYCurve,dScale*dCrvScale,pNewSurface));
          SER(pReg->GetBrep()->CreateFacesFromSurface(pNewSurface,
              pNewSurface->GetNaturalUVDomain(),SM_CT_G1,sFaces));
      }
  }
  return SM_SUCCESS;
} // end SmPrimitiveCreation::CreateSwungPrimitive

/*******************************************************************//**
PURPOSE: Creates a planar face from the given curves, and sweeps them,
            possibly into a solid. 

NOTES:
   The face may have internal loops.

   If the vertices/edges of the sweep happen to coincide with any vertices/edges
   of the Brep, they will be stitched together (cf SmStitch)

   The Brep is assumed to have a valid tolerance set.

   For now, the region must be the infinite region of the Brep
   (limitation of SmStitch).

   No curve must be degenerate.

   Curve endpoints must have exactly one match (within tolerance of Brep).

 
NOTE --- Alternate Routine: CreateTaperExtrude (below)
         allows for endcap selection, g1 discontinuity, and optional rounded corners

NOTE --- This routine does not use any SmTopologySweep object.  This means that
         it cannot set its entity maps, and cannot use a specialized StitchCallback.

***********************************************************************/
SmStatus SmPrimitiveCreation::CreateDraftSweep
 (SmTArray<SmCurve*> & cr3DCurves, // in : must be coplanar - consumed by this method.
  SmVector3d & crSweepVec,         // in : 
  double dSweepDist,               // in : degrees
  double dDraftAngleDeg,           // in : 0-90 inner, -90,0=outer                                  
  SmBoolean bCapEnds)              // in : 
{
  // empty entity maps - m_vMapToSame, m_vMapToHigher, m_vMapFromSame, and m_vMapFromLower
  InitEntityMaps();

  SmAxis2Placement sMove;
  SmVector3d sUnitSweep = crSweepVec;
  if (sUnitSweep.LengthSquared() < 0.00001)
      SmCurve::ComputeCrvsNormal(cr3DCurves, sUnitSweep);
  else sUnitSweep.Unitize();
  SmVector3d sTrans = dSweepDist * sUnitSweep;
  sMove.Translate(sTrans);

  SmTArray<SmCurve*> sCopyCrvs;
  for (ULONG ii=0; ii<cr3DCurves.GetSize(); ii++) {
      SmCurve *pCopy = NULL;
      cr3DCurves[ii]->Copy(*m_pRegion->GetContext(),pCopy);
      pCopy->Transform(sMove);
      sCopyCrvs.Add(pCopy);
  }

  double dOffset = smos_Tangent(dDraftAngleDeg*SM_PI/180.0) * dSweepDist;

  SmBrep* pBrep = m_pRegion->GetBrep();
  pBrep->m_bEditingEnabled = TRUE;
  double dTol = pBrep->GetTolerance();
  const SmContext * cpContext = pBrep->GetContext();

  SmBrep *pOffBrep = NULL;
  ULONG lSide = 1;
  if (dOffset < 0.0) lSide = 2;
  SmTArray<SmCurve*> sOwnedCopyCrvs(sCopyCrvs);
  SmObjsDelete<SmCurve*> sCopyCleanup(&sOwnedCopyCrvs);
  SmTArray<SmBoolean> sInputOwnershipTransferred;
  SmStatus eOffsetStat = SmPrimitiveCreation::OffsetProfile(
      *cpContext, sCopyCrvs, dTol, smos_Fabs(dOffset), lSide, FALSE, FALSE,
      pOffBrep, sInputOwnershipTransferred);
  for (ULONG ii=0;
       ii<sInputOwnershipTransferred.GetSize() && ii<sOwnedCopyCrvs.GetSize();
       ii++)
    {
      if (sInputOwnershipTransferred[ii])
        sOwnedCopyCrvs[ii] = NULL;
    }
  SER(eOffsetStat);
  SmObjDelete sCleanOff(pOffBrep);


  SmTArray<SmFace*> aFaces;
  SmTArray<SmEdge*> sEdges;
  if (pBrep->CreatePlanarFacesWith3DCurves(m_pRegion, cr3DCurves, dTol, aFaces) != SM_SUCCESS) {
      for (ULONG i=0; i<cr3DCurves.GetSize(); i++) {
          SmCurve *pCurve = cr3DCurves[i];
          SmEdge *pEdge = NULL;
          if (pBrep->CreateWireEdgeFromCurve(pCurve,pCurve->GetNaturalInterval(),pEdge) != SM_SUCCESS) {
              return SM_ERR;
          }
          sEdges.Add(pEdge);
      }
      aFaces.SetSize(0);
  }
  else {
      SmTArray<SmEdge*> sFaceEdges;
      for (ULONG i=0; i<aFaces.GetSize(); i++) {
          aFaces[i]->GetEdges(sFaceEdges);
          sEdges.Append(sFaceEdges);
      }
  }

  SmFace *pBaseFace = NULL;
  if (aFaces.GetSize() != 0) {
      pBaseFace = aFaces[0];
      pBaseFace->GetEdges(sEdges); 
  }
  else {
      bCapEnds = FALSE;
  }

  #ifdef SM_DEBUG_CODE
  SmBoolean bDebugMe = FALSE;
  if (bDebugMe) {
      smgfx_Erase();
      smgfx_SetLook(1,2, 1,0,0); pBrep->Draw();    sm_GraphicsLoop();
      smgfx_SetLook(1,2, 0,0,0); pOffBrep->Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop();
  }
  #endif // SM_DEBUG_CODE

  SER(pBrep->MergeBrep(*pOffBrep));
  SmTArray<SmEdge*> sAllEdges;
  pBrep->GetEdges(sAllEdges);

  for (ULONG jj=0; jj<sEdges.GetSize(); jj++) {
      SmEdge *pE = sEdges[jj];
      SmPoint3d sMidPnt;
      SER(pE->GetPrimaryEdgeuse()->NormalizedEvaluate(0.5,FALSE,sMidPnt));  // TRUE = UV Eval, FALSE = 3d Eval
      SmPoint3d sPnt = sMidPnt + dSweepDist * crSweepVec;
      SmEdge *pOffE = NULL;
      double dMinDist = SM_BIG_DOUBLE;
      for (ULONG kk=0; kk<sAllEdges.GetSize(); kk++) {
          SmEdge *pTestE = sAllEdges[kk];
          SmPoint3d sEPnt;
          double dParam, dDist;
          SmBoolean bSuccess;
          SER(pTestE->ClosestPoint(sPnt,bSuccess,dParam,dDist));
          if (bSuccess && dDist < dMinDist) {
              pOffE = pTestE;
              dMinDist = dDist;
          }
      }
      SmSurface *pRuled = NULL;
      SER(SmBrep::CreateEdgeEdgeBlend( *cpContext,                      // in : 
                                          0,                              // in : 0 = ruled surface
                                          pE->GetPrimaryEdgeuse(),        // in : 
                                          pOffE->GetPrimaryEdgeuse(),     // in : 
                                          1, NULL,                        // in : 1 = same curve dirs, NULL = no blend options
                                          pRuled));                       // out: 
      SmFace *pNewFace = NULL ;
      SER(pBrep->CreateFaceFromSurface(pRuled,pRuled->GetNaturalUVDomain(),pNewFace));
  }

  // Now create ruled surface between edges
      
  SmStitchCallback sStitchCallback;
  SmStitch sStitch(sStitchCallback,dTol,TRUE,FALSE);
  sStitch.m_bFastEdgeCompare = TRUE;
  sStitch.m_bSplitEdgesWithVertices = FALSE;
  sStitch.m_bDoRegionNesting = TRUE;
  double dMaxVGap, dMaxEGap;
  ULONG lNumStitched, lNumLamina;
  SER(sStitch.DoStitching(pBrep,NULL,NULL,lNumStitched,lNumLamina,dMaxVGap,dMaxEGap));

  pBrep->m_bEditingEnabled = FALSE;

  return SM_SUCCESS;

} // end SmPrimitiveCreation::CreateDraftSweep

/*****************************************************************
PURPOSE:  Used by CreateTaperExtrude (below)
  Return the joint point location between two links in a curve chain known to connect.

NOTES: 
  for 2 bspline curves that intersect at one start/end (which?)...return common intersection
  point....we cannot rely on orientation being head-tail....we ignore directions.

*****************************************************************/
static void sm_FindPivotPoint
 (SmBSplineCurve * pCrv1,       // in : one link in a chain of curve
  SmBSplineCurve * pCrv2,       // in : adjacent line in same chain
  double           dTol,        // in : Same point tol
  SmPoint3d      & PivotPoint)  // out: 3D location of the join point between the two links
{
  // locals 
  SmExtent1d sDomain1 = pCrv1->GetNaturalInterval();
  SmExtent1d sDomain2 = pCrv2->GetNaturalInterval();
  SmPoint3d  sEndPoint1, sStartPoint1;
  SmPoint3d  sEndPoint2, sStartPoint2;

  // Curve start points
  pCrv1->EvaluatePoint(sDomain1.GetMin(), sStartPoint1);
  pCrv2->EvaluatePoint(sDomain2.GetMin(), sStartPoint2);

  // coincident start points
  if ((sStartPoint1 - sStartPoint2).LengthSquared() < dTol)
    {
      PivotPoint = sStartPoint2;
      return;
    }

  // Curve1 end point
  pCrv1->EvaluatePoint(sDomain1.GetMax(), sEndPoint1);

  // Coincident Curve1EndPoint and Curve2StartPoint
  if ((sEndPoint1  -sStartPoint2).LengthSquared() < dTol)
    {
      PivotPoint = sStartPoint2;
      return;
    }
  
  // Must be Curve2 end Point 
  pCrv2->EvaluatePoint(sDomain2.GetMax(), PivotPoint);

  // all done
  return;

} // end SmPrimitiveCreation::sm_FindPivotPoint

/*****************************************************************
PURPOSE:  Utility function used by CreateTaperExtrude (below)
  Return a ruled surface between the offset curve pOffsetCrv and it's 
  generating curve in pComposite

NOTES: 
  For the offset curve method, this utility uses rUnMarriedCurves
  and rbMissingFaces.  For the surf xsection method they are 
  ignored.
*****************************************************************/
static SmStatus sm_CreateRuledSurfaceFromOffsetCurve 
(const SmContext                 & crContext,          // in : 
 const SmCurve                   * pOffsetCrv,         // in : Offset curve will be one edge of the ruled surface
 const SmTArray<SmBSplineCurve*> & crCurves,           // in : Curves being extruded
 SmCompositeCurve                * pComposite,         // in : ordered CompositeCurve of crCurves
 double                            dBrepTol,           // in : 
 double                            dSamePointTolerance,// in : Tolerance tighter than dBrepTol
 double                            dOffset,            // in : distance offset curves were set away
 SmAxis2Placement                  sMove,              // in : Translation to move offset curve for extrusion
 SmBoolean                         bSurfXSectMethod,   // in : TRUE : Calling this routine for the surf xsect method
                                                       //      FALSE: Calling this routing for the offset curve method
 SmTArray<SmBSplineCurve*>       & rUnMarriedCurves,   // i/o: List of generating curves tracks which have been used in offset curve method
 SmBoolean                       & rbMissingFaces,     // i/o: Does the offset curve method miss faces?
 SmBSplineSurface               *& rpRuledSurface)     // out: New ruled surface
{
  SmBSplineCurve *pBSC = SM_CAST_PTR(SmBSplineCurve , pOffsetCrv);

  // reduce the TargetCurve degree as much as possible
  if ( pBSC != NULL )
    { pBSC->DegreeReduction(dBrepTol , TRUE); }

  // Get the map attributes for this curve to marry curve segments with offset segments
  SmTArray<SmAttribute*> sAttributes;
  pOffsetCrv->GetAttributes(sAttributes);
  SmOffsetMapAttribute *pMap = (SmOffsetMapAttribute *)pOffsetCrv->FindAttribute(SM_AI_OFFSETMAP);
  if ( pMap == NULL )
    {
      WARN(_T("SmPrimitiveCreation::CreateTaperExtrude(): Curve without Map attribute.")) ;
      return SM_ERR;
    }

  // locals for mapping offset to original curves
  SmBoolean bCorner     = pMap->GetCornerOffset();   // TRUE = Curve is an offset of a corner between two curves
                                                     //        and not the offset of an original curve
  ULONG     lCurve      = pMap->GetCurve();          // Index to originating curve as indexed in the SmCompositeCurve array
  ULONG     lOtherCurve = pMap->GetOtherCurve();     // If this is a corner offset this is the index of the other curve

  // if a generating curve has already been made into a surface, prepare for surf xsect method
  if (   bSurfXSectMethod == FALSE
      && bCorner == FALSE 
      && rUnMarriedCurves[lCurve] == NULL )
    { rbMissingFaces = TRUE; }

#ifdef SM_DEBUG_CODE
  SmBoolean bLeftHand = pMap->GetLeftHandOffset(); // TRUE = curve results from a left handed offset.
  ULONG lCompIndex = pMap->GetCompositeIndex(); // index pointer to originating composite curve as sent into BuilCompositesFromCurves

  SmBoolean bDebugMe = FALSE;
  if ( bDebugMe )
    {
      TCHAR sBuff[SM_TBLOCK_SIZE];
      smos_sprintf(sBuff , _T("lCompIndex=%ld  bCorner=%d lCurve=%ld lOtherCurve=%ld lLH= %d\n") ,
                 lCompIndex , bCorner , lCurve , lOtherCurve , bLeftHand);
      smos_WriteBuffer(sBuff);
    }
#endif // SM_DEBUG_CODE

  // locals for upcoming RuledSurface call
  SmBSplineCurve *pRuleCurve1 = NULL , *pRuleCurve2 = NULL , *pRuleCurveOther = NULL;
  SmBSplineCurve *apData[3];
  SmTArray <SmBSplineCurve *> sTheseCurves(3 , apData);

  // use offset and original curves as rule curves for upcoming RuledSurface
  pRuleCurve1 = pBSC; // the new offset crv circle if corner 
  const SmCompositeCurveSegment *pSeg1 = pComposite->GetCurveSegment(lCurve);
  pRuleCurve2 = (SmBSplineCurve *)pSeg1->m_pParentCurve;

#ifdef SM_DEBUG_CODE     
  if ( bDebugMe )
    {
      pRuleCurve1->Dump(_T("RuleCurve1"));
      pRuleCurve2->Dump(_T("RuleCurve2"));
    }
#endif // SM_DEBUG_CODE

  // Get the SmBSplineCurveForm
  ULONG lDimension, lDegree;
  SmTArray<SmPoint3d> sControlPointList;
  SmBSplineCurveForm eBSCForm;
  SmTArray<ULONG> sKnotMult;
  SmTArray<double> sUniqueKnots;
  SmTArray<double> sWeights;
  SmKnotType eKnotT;
  SmObjDelete sCleanCurve;
  pRuleCurve2->GetCanonical( lDimension, lDegree, sControlPointList, eBSCForm, sKnotMult, sUniqueKnots, eKnotT, sWeights );

  // when offset curve was made from a corner between two curves and not from an original curve       
  if ( bCorner )
    {
      // find shared point from adjacent corner curves and make ruled surface from offset curve to point.
      // if this is a corner offset, there needs to be a lOtherCurve
      const SmCompositeCurveSegment *pSeg2 = pComposite->GetCurveSegment(lOtherCurve);
      pRuleCurveOther = (SmBSplineCurve *)pSeg2->m_pParentCurve;

      // get 3d Point between two original links
      SmPoint3d sPivotPoint;
      sm_FindPivotPoint(pRuleCurve2 ,          // in : one link in a chain of curve
                         pRuleCurveOther ,     // in : adjacent line in same chain
                         dSamePointTolerance , // in : Same point tol
                         sPivotPoint);         // out: 3D location of the join point between the two links

      // create degenerate curve at pivot point for upcoming ruledSurface call
      SmBSplineCurve *pLinePoint = NULL ;
      SER(SmBSplineCurve::CreatePointCurve(crContext , sPivotPoint, pLinePoint));  /* parameterized from 0 to 1 */
      sCleanCurve.SetObj( pLinePoint );

      // transform to final location and rule curve to point
      pRuleCurve1->Transform(sMove);
      sTheseCurves.Add(pRuleCurve1);
      sTheseCurves.Add(pLinePoint);
    }
  // Make proper cones, not degenerate past apex
  else if ( eBSCForm == SM_CF_CIRCULAR_ARC ) // || eBSCForm == SM_CF_ELLIPTIC_ARC )
  {
      // Determine if offset distance was greater than circle radius
      SmExtent1d sIvl  = pRuleCurve2->GetNaturalInterval();
      SmExtent1d sOIvl = pRuleCurve1->GetNaturalInterval();
      SmPoint3d  sCrvS, sCrvE, sOCrvS;
      double     dSS, dES;
      pRuleCurve2->EvaluatePoint( sIvl.GetMin(),  sCrvS );
      pRuleCurve2->EvaluatePoint( sIvl.GetMax(),  sCrvE );
      pRuleCurve1->EvaluatePoint( sOIvl.GetMin(), sOCrvS );
      dSS = sCrvS.DistanceBetweenSquared( sOCrvS );
      dES = sCrvE.DistanceBetweenSquared( sOCrvS );
      
      // dSS > dES implies offset was interior to the circle and past circle center
      if ( dSS > dES )
      {
          // Find center of the circle from three points
          SmPoint3d  sCrvM, sCenter;
          SmVector3d sXAxis, sYAxis;
          double     sRadius;
          ULONG      lDim = 3;
          SmBoolean  bClosed = FALSE;

          pRuleCurve2->EvaluatePoint( sIvl.GetMid(), sCrvM );
          smgu_CircleFrom3Points( sCrvS, sCrvM, sCrvE, sCenter, sXAxis, sYAxis, sIvl, sRadius, lDim, bClosed );

          // create degenerate curve at pivot point for upcoming ruledSurface call
          SmBSplineCurve *pLinePoint = NULL;
          SER( SmBSplineCurve::CreatePointCurve( crContext, sCenter, pLinePoint ) );  /* parameterized from 0 to 1 */
          sCleanCurve.SetObj( pLinePoint );
          
          SmPoint3d sO;
          sO.x = sRadius / dOffset * sMove.GetOrigin().x;
          sO.y = sRadius / dOffset * sMove.GetOrigin().y;
          sO.z = sRadius / dOffset * sMove.GetOrigin().z;
          SmAxis2Placement sTempMove(sO, sMove.GetXAxis(), sMove.GetYAxis() );
          pLinePoint->Transform( sTempMove );
          sTheseCurves.Add( pLinePoint );
          sTheseCurves.Add( pRuleCurve2 );

          rUnMarriedCurves[lCurve] = NULL; // mark this curve as paired - it's married now
      }
      else
      {
          pRuleCurve1->Transform( sMove );
          sTheseCurves.Add( pRuleCurve1 );
          sTheseCurves.Add( pRuleCurve2 );

          rUnMarriedCurves[lCurve] = NULL; // mark this curve as paired - it's married now
      }
  }
  else if ( eBSCForm == SM_CF_ELLIPTIC_ARC )
  {
      // This case is likely solvable similar to the above solution for SM_CF_CIRCULAR_ARC. Implement if needed. Needs 5 points?
      SM_DBG_WARN( _T( "Offsetting elliptic curve. If offset too far, self intersecting surfaces may arise." ) );
      pRuleCurve1->Transform( sMove );
      sTheseCurves.Add( pRuleCurve1 );
      sTheseCurves.Add( pRuleCurve2 );

      rUnMarriedCurves[lCurve] = NULL; // mark this curve as paired - it's married now
  }
  else // offset curve not a corner....just rule between curves transform offset to final location
    {
      pRuleCurve1->Transform(sMove);
      sTheseCurves.Add(pRuleCurve1);
      sTheseCurves.Add(pRuleCurve2);

      rUnMarriedCurves[lCurve] = NULL; // mark this curve as paired - it's married now
    }

  // If we're not using this result, all we needed was pRuleCurve1->Transform(sMove)
  if ( rbMissingFaces && !bSurfXSectMethod)
      return SM_ERR;

  // test for sliver surface (based on Face->IsDegenerate)
  if ( // both sides of a ruled surface are very small, delete it.
    (sTheseCurves[0]->ApproximateLength(crCurves[0]->GetNaturalInterval() , 3) < 2.*dBrepTol)
     && (sTheseCurves[1]->ApproximateLength(crCurves[1]->GetNaturalInterval() , 3) < 2.*dBrepTol) )
    {
      // skip sliver faces - leaving gaps in the final Brep but avoiding sliver faces
      return SM_ERR;
    }

  // make curves compatible - raise degrees and insert knots until both have the same knot vector
  SER(SmBSplineCurve::MakeCurvesCompatible(sTheseCurves , dSamePointTolerance));

  // If original curve was a corner, and the offset curve is rational (a circle),
  // then the degenerate curve for the corner must be set to have the same weights
  // as the offset curve, in order to create an SmCone.
  if (   bCorner
      && crCurves[0]->IsRational()
      && sTheseCurves.GetSize() == 2 )
    {
      // sort out the degenerate and the circle curves
      SmBSplineCurve *pDegenCurve = NULL;
      SmBSplineCurve *pOtherCurve = NULL;
      if ( sTheseCurves[0]->IsDegenerate() )
        {
          pDegenCurve = sTheseCurves[0];
          pOtherCurve = sTheseCurves[1];
        }
      else if ( sTheseCurves[1]->IsDegenerate() )
        {
          pDegenCurve = sTheseCurves[1];
          pOtherCurve = sTheseCurves[0];
        }
      // set the degenerate curve weights
      if ( pDegenCurve != NULL )
        {
          pDegenCurve->MatchWeights(pOtherCurve);
        }
    } // end circle offset from a chain corner check

#ifdef SM_DEBUG_CODE
  if ( bDebugMe )
    {
      // draw offset curves and cleaned up input curves
      //smgfx_Erase();
      smgfx_SetLook(4 , 5);
      sTheseCurves[0]->Draw(); sm_GraphicsLoop();
      sTheseCurves[1]->Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE
  // build a ruled surface for the [original, offset] curve pair
  //   note: If this ruling fails, the input curves are in opposite directions
  SER(SmBSplineSurface::CreateRuledSurface( crContext,       // in : context for new object construction 
                                           *sTheseCurves[0], // in : Shape for Min U or V surface isoparam curve 
                                           *sTheseCurves[1], // in : Shape for Max U or V surface isoparam curve 
                                            SM_SP_U,         // in : SM_SP_U = Surface U dir is linear, input curves vary in V
                                                             //      SM_SP_V = Surface V dir is linear, input curves vary in U
                                            rpRuledSurface));// out: The new ruled surface 
#ifdef SM_DEBUG_CODE
  if (bDebugMe) 
    {
      // draw offset curves and cleaned up input curves
      rpRuledSurface->Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // Pass attributes from offset curve to new surfaces for surf XSect method
  if( bSurfXSectMethod )
    { rpRuledSurface->AddAttribute(pMap); }

  return(SM_SUCCESS);
  
} // end SmPrimitiveCreation::sm_CreateRuledSurfaceFromOffsetCurve

/*******************************************************************//**
PURPOSE: Make a brep of Tapered surfaces from an array of 3D curves
            If endcaps are requested, will return a solid.
    
NOTES:
   The curves can be open or closed, need not be planar.
   If connected head to tail they must have a continuous direction.
     (Or they may be reversed on output.)
   The Height of the taper is the thickness of the solid(if endcaps).
   A negative height puts the taper on the side opposite the normal direction.
   The draft angle is measured from the normal(0) to the plane of the curves,
     typically 30 degrees for an inner taper, and -30 degrees for an outer taper.

   SmOffsetCornerType applies to outer corners only.
   Returns 1007 if self intersection problem on offset.

   This shrinks and stitches the brep before returning result.

METHOD SUMMARY ---
  Clean up the input curves - split curves at G1 discontinuities
                              order into chains
                              assemble into a composite curve
  CreateTrimmedOffsets - fill offset gaps with lines or fillets,
                         Trim overlapping offsets at offset intersection points,
                         remove short curves and fill in resulting gaps
  For every Original/Offset Curve Pair build and add to brep a ruled surface
  For every unPaired Original Curve Build a Curve/PivotPoint ruled surface
  For every unPaired Trim Curve Build a Curve/PivotPoint ruled surface
  When asked add an end cap over all original curves - 
           if original curves are not planar and closed this fails and the endCap is omitted from the output
  When asked add an end cap over all the offset curves -
           if offset curves are not planar and closed this fails and the endCap is omitted from the output
                              
IMPORTANT --- Cap construction fails for input curves that are not planar and closed.
   In which case a non Solid Brep will be constructed in which the end caps are missing.

   Optional Alternative to SmPrimitiveCreation::CreateDraftSweep.

   You must work with an independent set of 3D curves (no owner) and a new
   result brep.  The input r3DCurves get modified and incorporated into the
   result brep and will be deleted when that brep is deleted.

   The curves in r3DCurves are consumed here: they are either
   incorporated into the Brep (if an end cap is created at the
   input curves' end), or deleted.  So do not delete them after this call.

   If this call fails, however, the input curves are still valid, although
   they might have been modified: re-ordered, reversed, possibly deleted.

*********************************************************************/
SmStatus SmPrimitiveCreation::CreateTaperExtrude
 (SmTArray <SmCurve*> & r3DCurves,        // in : source curve (dir may get reversed if needed and not already head-tail) 
                                          //      note: 1. algorithm currently limited to curves of type SmBSplineCurve.
                                          //            2. These curves are consumed by this method.
  double                dHeight,          // in : + = direction of CrvsNormal, - = opp               
  double                dDraftAngleDeg,   // in : 0=along normal  0-90=inner 90-180=outer            
  int                   iEndCaps,         // in : 0=none, 1=at curves end, 2=at offset end, 3=both   
  SmVector3d          & rCrvsNormal,      // in : specified normal to plane of curves                 
                                          //    :  or enter [0,0,0] for us to compute it for you.    
  SmOffsetCornerType    eOffsetCorner)    // in : oneof SM_OC_LINEAR_EXTENSION = fill gaps made by offseting curves lines
                                          //            SM_OC_FILLET_CORNER    = fill gaps made by offseting curves with G1 fillet
                                          //    : default:[SM_OC_LINEAR_EXTENSION]                   
{
  // init SmPrimitiveCreation memory
  InitEntityMaps();

  // locals
  ULONG ii;
  SmBrep                      * pResultBrep = m_pRegion->GetBrep();
  const SmContext             * pContext    = pResultBrep->GetContext();
  SmBrep                      * pBrep       = new (*pContext) SmBrep();

  // make output temporary for now
  SmObjDelete sCleanBrep(pBrep);

  SmTArray <SmBSplineCurve *>   sCurves;          // input curves after being copied and split at G1 points
  SmTArray <SmBSplineCurve *>   sUnMarriedCurves; // original curves that aren't yet paired with offset curves
  SmTArray <SmCompositeCurve *> sCompositeCurves; // input sCurves after being organized into a chain, 
                                                  //   sCompositeCurves ref curves in sCurves - only delete once
  SmObjsDelete< SmCompositeCurve*> sCleanCCurvs( &sCompositeCurves );  // not same as deleting sCurves on exit 

  // check inputs
#ifdef SM_DEBUG_CODE
ULONG di ;                                                       
SmBoolean bDebugMe = FALSE ;
  if(bDebugMe)
    {
      SM_ASSERT_VALID(pResultBrep) ;
      for(di=0;di<r3DCurves.GetSize();di++)
        { SM_ASSERT_VALID(r3DCurves[di]) ; }

      SmVector3d sBase(0,0,0) ;
      if(r3DCurves.GetSize() > 0) r3DCurves[0]->EvaluatePoint(r3DCurves[0]->GetNaturalInterval().Evaluate(.5), sBase) ;

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,0) ; if(pResultBrep) pResultBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(2,3, 0,0,1) ; for(di=0;di<r3DCurves.GetSize();di++)
                                    { smgfx_ChangeColor(di!=0) ; r3DCurves[di]->Draw(NULL, TRUE) ; r3DCurves[di]->DrawParams() ; sm_GraphicsLoop() ; }
      smgfx_SetLook(4,5, 0,1,1) ; (dHeight * rCrvsNormal).Draw(&sBase) ; sm_GraphicsLoop() ; 
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // ready breps for modification
  SmTemporaryChangeValue<SmBoolean> sTemp(pResultBrep->m_bEditingEnabled, TRUE);
  pBrep->m_bEditingEnabled = TRUE;

  // Local tolerance uses
  SmZoneTol3d dBrepTol = SmTol::GetZoneTol3d( pResultBrep ); // Use Brep's own tol. [bd 090512]
  SmZoneTol3d dZoneTol = dBrepTol;
  double dApproxTol            = 0.001*pResultBrep->GetTolerance();
  double dSamePointTolerance   = 10.0*dApproxTol;
  pBrep->SetTolerance(dBrepTol);

  // Enforce head-tail order for input curves  ( nChain == number of chains found in r3DCurves array)
  SmTArray <ULONG> sChainStart;  
  SmBoolean        bDeleteDuplicates = TRUE;
  int              nChain            = SmCurve::FixChainCurvesList(r3DCurves,           // i/o: curves to order into chains (curve directions possibly reversed, duplicates possibly deleted)
                                                                   sChainStart,         // out: index of each chain start (chain includes curves from this index to next index or end of array)
                                                                                        //      rChainStart.GetSize() == number of chains in r3DCurves.
                                                                   2.*dZoneTol,         // in : dTol = max distance between matching curve endPoints
                                                                   bDeleteDuplicates ); // in : TRUE = Delete duplicate curves - set Curve's r3DCurves entry to NULL
                                                                                        //      FALSE= Don't (using duplicate curves may confuse the algorithm)
  SM_ASSERT(nChain == 1);

  // If the direction is given as [0,0,0] then compute normal from curve sample points
  if (rCrvsNormal.LengthSquared() < 0.00001)
    {
      SmCurve::ComputeCrvsNormal(r3DCurves, rCrvsNormal);

      // snap curves to Z axis when close
      // GWC: this is odd - either snap to all the coordinate axes or none
      if (rCrvsNormal.z >  0.98) { rCrvsNormal.x =  rCrvsNormal.y = 0.0; rCrvsNormal.z =  1.0;}  
      if (rCrvsNormal.z < -0.98) { rCrvsNormal.x =  rCrvsNormal.y = 0.0; rCrvsNormal.z = -1.0;}  
    }  
  
  else // a normal direction was supplied
    {
      // check that the curves order conforms to the specified input normal
      SmPoint3d sCrvNormal;
      SmCurve::ComputeCrvsNormal(r3DCurves, sCrvNormal);
 
      // get angle between computed and given normal running from 0 to Pi in radians
      double dAngleBetweenRad = 0.0;
      sCrvNormal.AngleBetween(rCrvsNormal, dAngleBetweenRad);
      
      // if ComputedNormal is not consistent with the specified normal, reverse all the curves 
      if(fabs(dAngleBetweenRad - SM_PI) < 0.0001)
        {  
          // just the 1st is not enough...there may be more than 1 chain
          for (ii=0; ii < r3DCurves.GetSize(); ii++)
            {
              SmCurve *pCrv = r3DCurves[ii];
              SmExtent1d sIvl = pCrv->GetNaturalInterval();
              pCrv->ReverseParameterization(sIvl, sIvl);
            }
          
          // rechain all the curves    
          SmTArray <ULONG> sStartChain; //ignore
          SmCurve::FixChainCurvesList(r3DCurves,    // i/o: curves to order into chains (curve directions possibly reversed, duplicates possibly deleted)
                                      sStartChain,  // out: index of each chain start (chain includes curves from this index to next index or end of array)
                                                    //      rChainStart.GetSize() == number of chains in r3DCurves.
                                      2.*dZoneTol,  // in : dTol = max distance between matching curve endPoints
                                      TRUE );       // in : TRUE = Delete duplicate curves
        }                                           //      FALSE= Don't (using duplicate curves may confuse the algorithm)
    } // end specified normal branch
  
  // build translation to generate the offset curves
  SmAxis2Placement sMove;
  SmVector3d       sTrans = dHeight * rCrvsNormal;
  sMove.Translate(sTrans);
  
  // select Left or Right hand offset 
  SmOffsetDirectionType eOffsetDir =(dDraftAngleDeg < 0) ? SM_OD_RIGHT_HAND_SIDE : SM_OD_LEFT_HAND_SIDE; 
  
  // This is the offset distance in the plane of the input curves    
  double dOffset = dHeight*(smos_Tangent(fabs(dDraftAngleDeg)*SM_PI/180.0));          
  if (dOffset < 0.0)
      dOffset = -dOffset;
  
  // Break up all input curves at G1 discontinuities into sCurves:[local copies]
  for (ii=0; ii < r3DCurves.GetSize(); ii++)
    {
      SM_ASSERT(r3DCurves[ii]->IsKindOf(SmBSplineCurve_TYPE));

      SmBSplineCurve *pBSC =(SmBSplineCurve *)r3DCurves[ii];
      SmTArray < SmBSplineCurve *> sG1Curves;
      pBSC->SubdivideAtDiscontinuity(*pContext,  // in : context for new object construction
                                     SM_CT_G1,   // in : curve is split at internal continuities <= to this value
                                     sG1Curves); // out: newly copied curves - may be 1 curve copy when no internal discontinuities

      // load results into working arrays
      sCurves.Append(sG1Curves);          // list of all split at G1 copied curves
      sUnMarriedCurves.Append(sG1Curves); // all copied curves are yet to be paired with offset curves
    } 

  // Approx tol has to be VERY small to return very small self-intersection results
  double dDistanceToCreateLine = 0.0;

  // connect sCurves into composite curves - ownership moves from sCurves to sCompCurves array
  SmCompositeCurve::BuildCompositesFromCurves
    (*pContext,             // in : context for new object construction
     sCurves,               // in : unordered curves to connect into composites
                            //      note: Curve ownership is NOT moved to the output composites.
                            //            delete these after deleting the composite.
     FALSE,                 // in : (not yet used) bMakeCurvesHomogeneous : TRUE = Approx curves with deg 3 NUBs
     dApproxTol,            // in : (not yet used) dThisApproxTol3d = tol used if an approximation is required.
     dSamePointTolerance,           // in : if      gap < sXSectTol3d, connect
     0.0,                   // in : (not yet used) else if gap < dOptDistanceToAverage,    snap to avg and connect - not implemented
     0.0,                   // in : (not yet used) else if gap < dOptDistanceToExtendTrim, trim and connect        - not implemented
     dDistanceToCreateLine, // in : else if gap < dOptDistanceToCreateLine, insert line seg and connect, 0.0 to ignore
     0.0,                   // in : (not yet used) else if gap < dOptDistanceToCreateBlend, insert blend and connect
     sCompositeCurves);     // out: Composite Curve(s) ref unordered crCurves[i] as segments but don't own them.
                            //      They may also contain and own NewCurves used to fill gaps which are deleted when they are deleted. 
                            //      crCurves must exist as long as these objs and their mem must
                            //      be managed after the Composites are deleted.
  
  // expect one composite curve
  SM_ASSERT_MSG(sCompositeCurves.GetSize() == 1,
                _T("CreateTaperExtrude - input curves don't connect into 1 CompositeCurve - output won't be manifold")) ;                                                                                                                                                            
  SmCompositeCurve* pComposite = sCompositeCurves[0]; 

#ifdef SM_DEBUG_CODE
  if(bDebugMe)
    {
      SM_DUMP_AND_ASSERT_VALID(pComposite) ;

      SmVector3d sBase(0,0,0) ;
      if(r3DCurves.GetSize() > 0) r3DCurves[0]->EvaluatePoint(r3DCurves[0]->GetNaturalInterval().Evaluate(.5), sBase) ;

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,1,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(2,3, 0,0,1) ; pComposite->Draw(NULL, TRUE) ; pComposite->DrawParams() ; sm_GraphicsLoop() ; 
      smgfx_SetLook(4,5, 0,1,1) ; (dHeight * rCrvsNormal).Draw(&sBase) ; sm_GraphicsLoop() ; 
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // offset the composite curve - extend convex joints and intersect concave ones.
  //   output curves will have attributes (SmOffsetMapAttribute) that describe
  //   where the output curve came from and how it was generated.    
  SmTArray <SmBSplineCurve*> sTrimmedOffsets;

  // GWC_NEEDS_WORK GET_BETTER_VALUES_FOR_TOLERANCE_IN_FOLLOWING_CALL GWC_LINE ;
  pComposite->CreateTrimmedOffsets    // eff: call CreateOffsetsOfManyCurves() for this curve.
    (*pContext,                       // in : context for new object construction
     2.*dZoneTol,                     // in : Minimum distance at which offset curve end-gaps are filled with corner curves
     dApproxTol,                      // in : Tolerance to which BSpline Approximations to exact offset curves are built
     rCrvsNormal,                     // in : Defines, along with the curve's parameter direction,
                                      //      the right and left hand offset directions.
     eOffsetCorner,                   // in : SM_OC_LINEAR_EXTENSION: corner = 2 lines from given ends to common linear extension xsect point.
                                      //      SM_OC_FILLET_CORNER   : corner = fillet arc centered on crVertexPoint running to given end points
                                      //      SM_OC_LINEAR_CHAMFER  : corner = line between given end points (result is actually within offset distance so bTrimResults must = False)
     eOffsetDir,                      // in : Corresponding direction of each composite
                                      //      curve member to offset.  1-LEFT, 2-RIGHT, 3-BOTH
     TRUE,                            // in : TRUE = concave raw offsets are intersected and trimmed back to common intersection points
                                      //      FALSE= skip trim step
     dOffset,                         // in : offset distance (a negative value negates the offset direction)
     sTrimmedOffsets);                // out: Resulting new offset curves will have attribute attached
                                      //      describing origination of curve
                                      // in : TRUE = if BSplineCurve just copy it (preserves CrvParams)
                                      //      FALSE= approximate BSplineCurves (changes CrvParams), NonBSplineCrvs always Approximated
                                      // in : default:[FALSE]

  // arrive here when - this method owns all the curves in
  //   3DCurves - remaining nonDegenerate input curves
  //   sCurves  - 3DCurves, copied and split at G1. 
  //              Curves are referenced unowned in sComposites. sComposites may own additional gap filling lines.
  //   sTrimmedOffsets - offsets of sCurves (not yet moved to final positions)
  
  SmBoolean bMissingFaces = FALSE;
  SmBoolean bEmptyOffsets = FALSE;
  SmTArray<SmCurve *> sTrimCrvs;  // pass bsplines as curves (sTrimCrvs == sTrimmedOffsets)

  // expect some TrimmedOffset curves as output
  if(sTrimmedOffsets.GetSize() < 1) 
    { 
      bMissingFaces = TRUE; 
      bEmptyOffsets = TRUE;
    }
  else{
#ifdef SM_DEBUG_CODE
  if (bDebugMe) 
    {
      // draw input curves   pComposite
      ULONG lDim ;
      SmTArray<SmCurve*> sOrigCurves ;
      SmBoolean          bClosed ;
      pComposite->GetCanonical(lDim, sOrigCurves, bClosed) ;

      smgfx_Erase() ;
      smgfx_ChangeColor(FALSE) ; 
      for(di=0;di<sOrigCurves.GetSize();di++)
        { SmCurve *pCurve = sOrigCurves[di] ;
          if(pCurve) { SmExtent1d sIvl = pCurve->GetNaturalInterval() ;
                       SmVector3d sStartPt, sEndPt ;
                       pCurve->EvaluatePoint( sIvl.GetMin(), sStartPt) ;
                       pCurve->EvaluatePoint( sIvl.GetMax(), sEndPt) ;
                       smgfx_SetLook(2,3) ; smgfx_ChangeColor(TRUE) ; pCurve->Draw() ; sm_GraphicsLoop() ;
                       smgfx_SetLook(3,4, 1,0,0) ; sStartPt.Draw() ; sm_GraphicsLoop() ;
                       smgfx_SetLook(5,6, 0,0,1) ; sEndPt.Draw() ; sm_GraphicsLoop() ;
                     }
        } 
      sm_GraphicsLoop() ;

      // draw offset curves
      smgfx_ChangeColor(FALSE) ; 
      for(di=0;di<sTrimmedOffsets.GetSize();di++)
        { SmCurve *pCurve = sTrimmedOffsets[di] ;
          if ( pCurve != NULL )
            {
              SmExtent1d sIvl = pCurve->GetNaturalInterval() ;
              SmVector3d sStartPt, sEndPt ;
              pCurve->EvaluatePoint( sIvl.GetMin(), sStartPt) ;
              pCurve->EvaluatePoint( sIvl.GetMax(), sEndPt) ;
              smgfx_SetLook(2,3) ; smgfx_ChangeColor(TRUE) ; pCurve->Draw() ; sm_GraphicsLoop() ;
              smgfx_SetLook(3,4, 1,0,0) ; sStartPt.Draw() ; sm_GraphicsLoop() ;
              smgfx_SetLook(5,6, 0,0,1) ; sEndPt.Draw() ; sm_GraphicsLoop() ;
            }
        } 
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // reorder the TrimmedOffset curves back into a chain
  for(ii=0;ii<sTrimmedOffsets.GetSize();ii++)
    {
      sTrimCrvs.Add((SmCurve *)sTrimmedOffsets[ii]) ;
    }
  nChain  = SmCurve::FixChainCurvesList(sTrimCrvs,   // i/o: curves to order into chains (curve directions possibly reversed, duplicates possibly deleted)
                                        sChainStart, // out: index of each chain start (chain includes curves from this index to next index or end of array)
                                                     //      rChainStart.GetSize() == number of chains in r3DCurves.
                                        2.*dZoneTol, // in : dTol = max distance between matching curve endPoints
                                        TRUE );      // in : TRUE = Delete duplicate curves
                                                     //      FALSE= Don't (using duplicate curves may confuse the algorithm)
                                                     //      default:[TRUE]
  SM_ASSERT_MSG(sChainStart.GetSize()==sCompositeCurves.GetSize(),
                _T("CreateTaperExtrude - created offset curves that don't keep input CompositeCurves connected")) ;

  // Tightened tolerance [B632]
  //// close resulting gaps
  //// use a big value to define the size of a gap to be closed by endPoint control point moves
  //// but if the value is bigger than half the offset distance, then adjust down
  //double dGapTol = 0.025*fabs(dHeight);  
  //if( dGapTol > dOffset / 2 )
  //    dGapTol = dOffset / 2;   // gwc: why such a large tolerance? shouldn't XSectTol3d be ok? This tol use is an upper bound
  //                             //      and we want to fix cases where offset gaps happen to exceed XSectTol3d.
  //                             //      I guess it's okay to leave this a very large value here.

  // move SmBSplineCurve EndPts to close small gaps in a head to toe curve sequence
  // GWC_NEEDS_WORK change_dGapTol_HERE_TO_BE_sXSectTol3d__Check_number_of_chains_DONE__should_be_Same_as_sComposites__one_for_Manifold GWC_LINE ;

  SmCurve::CrvsFixGaps( sTrimCrvs,    // i/o: curves must be of type SmBSplineCurve
                       2.*dZoneTol ); // in : Crvs are matched when endPtGap < sXSectTol3d
                                      // note: closes curveEnd/CurveEnd gaps - does not remove short curves
#ifdef SM_DEBUG_CODE
  if (bDebugMe) 
    {
      // draw offset curves and cleaned up input curves
      smgfx_Erase();
      smgfx_SetLook( 1,2); 
      for(di=0;di<sTrimCrvs.GetSize();di++)        { SmCurve *pC =(SmBSplineCurve *)sTrimCrvs[di];
                                                     pC->Dump() ;
                                                     smgfx_ChangeColor(di!=0); pC->Draw(); sm_GraphicsLoop();
                                           }
      for(di=0;di<sCompositeCurves.GetSize();di++) { SmCompositeCurve *pC =sCompositeCurves[di];
                                                     pC->Dump() ;
                                                     smgfx_ChangeColor(di!=0); pC->Draw(); sm_GraphicsLoop();
                                           }
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // for every offset curve - find its original partner and build a Ruled Surface between the two curves
  for (ii=0;ii<sTrimCrvs.GetSize();ii++)
    {
      SmCurve        *pOffsetCrv = sTrimCrvs[ii];
      
      SmBSplineSurface *pRuledSurface = NULL;
      if ( SM_SUCCESS != sm_CreateRuledSurfaceFromOffsetCurve(*pContext,              // in : 
                                                               pOffsetCrv,            // in : Offset curve will be one edge of the ruled surface
                                                               sCurves,               // in : Curves being extruded
                                                               pComposite,            // in : ordered CompositeCurve of sCurves
                                                               dZoneTol,              // in : 
                                                               dSamePointTolerance,   // in : Tolerance tighter than dZoneTol
                                                               dOffset,
                                                               sMove,                 // in : Translation to move offset curve for extrusion
                                                               FALSE,                 // in : We are using the trimmed offset curve method
                                                               sUnMarriedCurves,      // i/o: List of generating curves tracks which have been used
                                                               bMissingFaces,         // i/o: Does the offset curve method miss faces?
                                                               pRuledSurface ) )      // out: New ruled surface
      { continue; }
  
      // create a face from the ruled surface placed in pBrep Infinite region
      SmFace *pNewFace = NULL ;

      if ( pRuledSurface != NULL )
        {
          pBrep->CreateFaceFromSurface(pRuledSurface,                       // in : target face
                                       pRuledSurface->GetNaturalUVDomain(), // in : desired face Nurb domain
                                       pNewFace);                           // out: New Face connected to this Brep
        }
      else
        { WARN(_T("SmPrimitiveCreation::CreateTaperExtrude(): pRuledSurface creation failed.")) ; }

#ifdef SM_DEBUG_CODE
      // draw target NewFace(black) pRuledSurface (yellow).
      if (bDebugMe)
        {
          SM_DUMP_AND_ASSERT_VALID(pBrep) ;

          smgfx_Erase();
          smgfx_SetLook(1,2, 0,0,0 ); if(pBrep) pBrep->Draw(TRUE); sm_GraphicsLoop();
          smgfx_SetLook(1,2, 0,0,0) ; if(pNewFace) pNewFace->DrawUV() ; sm_GraphicsLoop();
          smgfx_SetLook(1,4, 1,1,0) ; if(pRuledSurface) pRuledSurface->DrawUV(4,4); sm_GraphicsLoop();
          smgfx_SetLook(3,6, 1,0,0) ; if(pRuledSurface) pRuledSurface->DrawPoles(); sm_GraphicsLoop();
          smgfx_SetLook(3,6, 1,0,0) ; if(pRuledSurface) pRuledSurface->DrawSeams(); sm_GraphicsLoop();
          smgfx_SetLook(1,4, 1,0,0) ; if(pRuledSurface) pRuledSurface->DrawVectorField(pRuledSurface->GetNaturalUVDomain(), 4, 4, SM_DM_UNIT_NORMAL) ; sm_GraphicsLoop() ;

          // draw offset curves and cleaned up input curves
          smgfx_SetLook( 1,2); 
          for(di=0;di<sTrimCrvs.GetSize();di++)        { SmCurve *pC =(SmBSplineCurve *)sTrimCrvs[di];
                                                         pC->Dump() ;
                                                         smgfx_ChangeColor(di!=0); pC->Draw(); sm_GraphicsLoop();
                                                       }
          for(di=0;di<sCompositeCurves.GetSize();di++) { SmCompositeCurve *pC =sCompositeCurves[di];
                                                         pC->Dump() ;
                                                         smgfx_ChangeColor(di!=0); pC->Draw(); sm_GraphicsLoop();
                                                       }
          sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE - draw inputs

    } // end iter every offset and trimmed curve
  
  // UnMarriedCurves exist when the offest curve was completely trimmed away
  // If there are UnMarriedCurves: 
  //  NEW METHOD: use surface intersection method since there are missing faces.
  //  OLD METHOD: connect original unmarried segment to nearby offset curve end point. 

  // for every entry in the UnMarriedCurves array
  for(ii=0 ; ii<sUnMarriedCurves.GetSize() && !bMissingFaces ; ii++)
    {
      SmBSplineCurve *pCrv1 = sUnMarriedCurves[ii];

      // skip NULL entries - those curves have already been married and used to build ruled surfaces
      if (pCrv1 == NULL)
        { continue; }
      else
        { bMissingFaces = TRUE; }

    } // end iter unmarried original curves 

  // Stitch the potential result to get the topology
  ULONG  lNumStitched, lNumLamina;
  double dMaxVGap = 0.0, dMaxEGap = 0.0;
  pBrep->StitchFaces(dBrepTol, lNumStitched, lNumLamina, dMaxVGap, dMaxEGap); 

#ifdef SM_DEBUG_CODE
  if ( bDebugMe )
  {
      smgfx_Erase();
      pBrep->Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop();
  }
#endif // SM_DEBUG_CODE

  // Check the potential result for missing faces
  SmTArray<SmFace*> sFaces;
  pBrep->GetFaces(sFaces);

  // Each face should be lamina along input curve and offset curve, manifold otherwise
  for (ii = 0 ; ii < sFaces.GetSize() && !bMissingFaces ; ii++)
    {
      // Walk around outer loop of face. Return True if 2 consecutive edges are lamina.
      bMissingFaces = (sFaces[ii]->HasConsecutiveLaminaEdges()) ? TRUE : FALSE ;
    }
}
  // if the result has struts or is empty, assume it's missing faces and redo w/ high-work surface intersection method
  if(!bMissingFaces)
    { 
      SmTArray<SmFace*> sFaces;
      pBrep->GetFaces(sFaces);
      pBrep->CopyFaces(sFaces, pResultBrep, NULL, FALSE);
    } // end no missing faces
  else  {

      // Create local breps to instersect surfaces
      SmBrep * pSurfBrep = new (*pContext) SmBrep();
      SmBrep * pSurfXSectBrep = new (*pContext) SmBrep();
      SmObjDelete sCleanSurfBrep( pSurfBrep );

      // Prepare breps for appropriate editing
      pSurfBrep->m_bEditingEnabled      = TRUE;
      pSurfXSectBrep->m_bEditingEnabled = TRUE;
      pSurfBrep->SetTolerance(dBrepTol);
      pSurfXSectBrep->SetTolerance(dBrepTol);

      // offset the composite curve - extend convex joints and intersect concave ones.
      //   output curves will have attributes (SmOffsetMapAttribute) that describe
      //   where the output curve came from and how it was generated.    
      SmTArray <SmBSplineCurve*> sUnTrimmedOffsets;
      SmObjsDelete< SmBSplineCurve*> sCleanTrimmedBSCurves( &sUnTrimmedOffsets );

      // GWC_NEEDS_WORK GET_BETTER_VALUES_FOR_TOLERANCE_IN_FOLLOWING_CALL GWC_LINE ;
      pComposite->CreateTrimmedOffsets  // eff: call CreateOffsetsOfManyCurves() for this curve.
      (*pContext,                       // in : context for new object construction
       2.*dZoneTol,                        // in : Minimum distance at which offset curve end-gaps are filled with corner curves
       dApproxTol,                      // in : Tolerance to which BSpline Approximations to exact offset curves are built
       rCrvsNormal,                     // in : Defines, along with the curve's parameter direction,
                                        //      the right and left hand offset directions.
       eOffsetCorner,                   // in : SM_OC_LINEAR_EXTENSION: corner = 2 lines from given ends to common linear extension xsect point.
                                        //      SM_OC_FILLET_CORNER   : corner = fillet arc centered on crVertexPoint running to given end points
                                        //      SM_OC_LINEAR_CHAMFER  : corner = line between given end points (result is actually within offset distance so bTrimResults must = False)
       eOffsetDir,                      // in : Corresponding direction of each composite
                                        //      curve member to offset.  1-LEFT, 2-RIGHT, 3-BOTH
       FALSE,                           // in : TRUE = concave raw offsets are intersected and trimmed back to common intersection points
                                        //      FALSE= skip trim step
       dOffset,                         // in : offset distance (a negative value negates the offset direction)
       sUnTrimmedOffsets);              // out: Resulting new offset curves will have attribute attached
                                        //      describing origination of curve
                                        // in : TRUE = if BSplineCurve just copy it (preserves CrvParams)
                                        //      FALSE= approximate BSplineCurves (changes CrvParams), NonBSplineCrvs always Approximated
                                        // in : default:[FALSE]

      // arrive here when - this method owns all the curves in
      //   3DCurves - remaining nonDegenerate input curves
      //   sCurves  - 3DCurves, copied and split at G1. 
      //              Curves are referenced unowned in sComposites. sComposites may own additional gap filling lines.
      //   sUnTrimmedOffsets - offsets of sCurves (not yet moved to final positions)

      // expect some UnTrimmedOffset curves as output
      if (sUnTrimmedOffsets.GetSize() < 1)
        { return (SM_ERR) ; }

#ifdef SM_DEBUG_CODE
      if (bDebugMe)
        {
        // draw input curves   pComposite
        ULONG lDim ;
        SmTArray<SmCurve*> sOrigCurves ;
        SmBoolean          bClosed ;
        pComposite->GetCanonical(lDim, sOrigCurves, bClosed) ;

        smgfx_Erase() ;
        smgfx_ChangeColor(FALSE) ;
        for (di = 0; di < sOrigCurves.GetSize(); di++)
          {
          SmCurve *pCurve = sOrigCurves[di] ;
          if (pCurve)
            {
            SmExtent1d sIvl = pCurve->GetNaturalInterval() ;
            SmVector3d sStartPt, sEndPt ;
            pCurve->EvaluatePoint(sIvl.GetMin(), sStartPt) ;
            pCurve->EvaluatePoint(sIvl.GetMax(), sEndPt) ;
            smgfx_SetLook(2, 3) ; smgfx_ChangeColor(TRUE) ; pCurve->Draw() ; sm_GraphicsLoop() ;
            smgfx_SetLook(3, 4, 1, 0, 0) ; sStartPt.Draw() ; sm_GraphicsLoop() ;
            smgfx_SetLook(5, 6, 0, 0, 1) ; sEndPt.Draw() ; sm_GraphicsLoop() ;
            }
          }
        sm_GraphicsLoop() ;

        // draw offset curves
        smgfx_ChangeColor(FALSE) ;
        for (di = 0; di < sUnTrimmedOffsets.GetSize(); di++)
          {
          SmCurve *pCurve = sUnTrimmedOffsets[di] ;
          if (pCurve != NULL)
            {
            SmExtent1d sIvl = pCurve->GetNaturalInterval() ;
            SmVector3d sStartPt, sEndPt ;
            pCurve->EvaluatePoint(sIvl.GetMin(), sStartPt) ;
            pCurve->EvaluatePoint(sIvl.GetMax(), sEndPt) ;
            smgfx_SetLook(2, 3) ; smgfx_ChangeColor(TRUE) ; pCurve->Draw() ; sm_GraphicsLoop() ;
            smgfx_SetLook(3, 4, 1, 0, 0) ; sStartPt.Draw() ; sm_GraphicsLoop() ;
            smgfx_SetLook(5, 6, 0, 0, 1) ; sEndPt.Draw() ; sm_GraphicsLoop() ;
            }
          }
        sm_GraphicsLoop() ;
        }
#endif // SM_DEBUG_CODE

      SmTArray<SmCurve *> sUnTrimCrvs;  // pass bsplines as curves (sUnTrimCrvs == sUnTrimmedOffsets)
      for (ii = 0; ii < sUnTrimmedOffsets.GetSize(); ii++)
        {
          sUnTrimCrvs.Add((SmCurve *)sUnTrimmedOffsets[ii]) ;
        }

#ifdef SM_DEBUG_CODE
      if (bDebugMe)
        {
        // draw offset curves and cleaned up input curves
        smgfx_Erase();
        smgfx_SetLook(1, 2);
        for (di = 0; di < sUnTrimCrvs.GetSize(); di++)
          {
          SmCurve *pC = (SmBSplineCurve *)sUnTrimCrvs[di];
          pC->Dump() ;
          smgfx_ChangeColor(di != 0); pC->Draw(); sm_GraphicsLoop();
          }
        for (di = 0; di < sCompositeCurves.GetSize(); di++)
          {
          SmCompositeCurve *pC = sCompositeCurves[di];
          pC->Dump() ;
          smgfx_ChangeColor(di != 0); pC->Draw(); sm_GraphicsLoop();
          }
        sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE

      // for every offset curve - find its original partner and build a Ruled Surface between the two curves
      for (ii = 0; ii < sUnTrimCrvs.GetSize(); ii++)
        {
          SmCurve        *pOffsetCrv = sUnTrimCrvs[ii];

          SmBSplineSurface *pRuledSurface = NULL;
          if ( SM_SUCCESS != sm_CreateRuledSurfaceFromOffsetCurve(*pContext,              // in : 
                                                                   pOffsetCrv,            // in : Offset curve will be one edge of the ruled surface
                                                                   sCurves,               // in : Curves being extruded
                                                                   pComposite,            // in : ordered CompositeCurve of sCurves
                                                                   dZoneTol,              // in : 
                                                                   dSamePointTolerance,   // in : Tolerance tighter than dZoneTol
                                                                   dOffset,
                                                                   sMove,                 // in : Translation to move offset curve for extrusion
                                                                   TRUE,                  // in : We are using the surface xsect method
                                                                   sUnMarriedCurves,      // i/o: List of generating curves tracks which have been used
                                                                   bMissingFaces,         // i/o: Does the offset curve method miss faces?
                                                                   pRuledSurface ) )      // out: New ruled surface
          { continue; }

          // create a face from the ruled surface placed in pSurfXSectBrep Infinite region
          SmFace *pNewFace = NULL ;

          if ( pRuledSurface != NULL )
            {
              pSurfBrep->CreateFaceFromSurface(pRuledSurface,                       // in : target face
                                               pRuledSurface->GetNaturalUVDomain(), // in : desired face Nurb domain
                                               pNewFace);                           // out: New Face connected to this Brep
            }
          else
            { WARN(_T("SmPrimitiveCreation::CreateTaperExtrude(): pRuledSurface creation failed.")) ; }

#ifdef SM_DEBUG_CODE
          // draw target NewFace(black) pRuledSurface (yellow).
          if (bDebugMe)
            {
              SM_DUMP_AND_ASSERT_VALID(pSurfBrep) ;

              smgfx_Erase();
              smgfx_SetLook(1, 2, 0, 0, 0); if (pSurfBrep) pSurfBrep->Draw(TRUE); sm_GraphicsLoop();
              smgfx_SetLook(1, 2, 0, 0, 0) ; if (pNewFace) pNewFace->DrawUV() ; sm_GraphicsLoop();
              smgfx_SetLook(1, 4, 1, 1, 0) ; if (pRuledSurface) pRuledSurface->DrawUV(4, 4); sm_GraphicsLoop();
              smgfx_SetLook(3, 6, 1, 0, 0) ; if (pRuledSurface) pRuledSurface->DrawPoles(); sm_GraphicsLoop();
              smgfx_SetLook(3, 6, 1, 0, 0) ; if (pRuledSurface) pRuledSurface->DrawSeams(); sm_GraphicsLoop();
              smgfx_SetLook(1, 4, 1, 0, 0) ; if (pRuledSurface) pRuledSurface->DrawVectorField(pRuledSurface->GetNaturalUVDomain(), 4, 4, SM_DM_UNIT_NORMAL) ; sm_GraphicsLoop() ;

              // draw offset curves and cleaned up input curves
              smgfx_SetLook(1, 2);
              for (di = 0; di < sUnTrimCrvs.GetSize(); di++)
                {
                  SmCurve *pC = (SmBSplineCurve *)sUnTrimCrvs[di];
                  pC->Dump() ;
                  smgfx_ChangeColor(di != 0); pC->Draw(); sm_GraphicsLoop();
                }
              for (di = 0; di < sCompositeCurves.GetSize(); di++)
                {
                  SmCompositeCurve *pC = sCompositeCurves[di];
                  pC->Dump() ;
                  smgfx_ChangeColor(di != 0); pC->Draw(); sm_GraphicsLoop();
                }
              sm_GraphicsLoop();
            }
#endif // SM_DEBUG_CODE - draw inputs

        } // end iter every offset and trimmed curve

      // Piecewise Merge each face into the Brep
      SmMerge sMergeExec(*pContext, pSurfXSectBrep, pSurfBrep, dBrepTol);
      SER( sMergeExec.PiecewiseMerge( NULL, TRUE, TRUE, pSurfXSectBrep ) );

      /* If this boolean fails, it's possibly due to "too large" offset distances.  
         Offsets greater than curvature^-1 will cause kinks in the offset curve.
         If this becomes a problem we can try to implement a third method that 
         calculates curvature on input curves.  We would then cut the original curves 
         where dOffset == 1/curvature. Consider a curve with straight ends and a high
         curvature mid; this would be cut into 3 intervals. On the straight ends where 
         the Ivls produce valid outputs, keep those. Those two valid outputs should meet.
         Use the meeting point as a degenerate edge to connect to the high curvature Ivl
         as the other end of the ruled surface.
      */

      // Get the faces in the Brep
      SmTArray<SmFace*> sFaces;
      pSurfXSectBrep->GetFaces(sFaces);

      // Iterate over Faces to remove extraneous faces
      for (ii = 0 ; ii < sFaces.GetSize() ; ii++)
        {
          // Face loop locals
          SmFace  * pFace         = sFaces[ii];
          SmBoolean bConnectStart = FALSE; 
          SmBoolean bConnectEnd   = FALSE;

#ifdef SM_DEBUG_CODE
          if (bDebugMe)
            {
              smgfx_Erase();
              smgfx_SetLook(1,2, 0,1,1) ; if(pSurfXSectBrep) pSurfXSectBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
              smgfx_SetLook(2,3, 0,0,1) ; pFace->Draw() ; sm_GraphicsLoop() ;
              sm_GraphicsLoop() ;
            }
#endif // SM_DEBUG_CODE

          // Get the map attributes for the curve that generated pFace
          SmOffsetMapAttribute *pMap = (SmOffsetMapAttribute *)pFace->GetSurface()->FindAttribute(SM_AI_OFFSETMAP);

          // Index to originating curve as indexed in the SmCompositeCurve array
          ULONG     lCurve  = pMap->GetCurve();
          SmBoolean bCorner = pMap->GetCornerOffset();

          // Parent curve (or connected vertex) that generated this surface
          SmCurve*  pParentCurve = (SmCurve *)pComposite->GetCurveSegment(lCurve)->m_pParentCurve;
          SmPoint3d sParentStart, sParentEnd;
          pParentCurve->GetEnds(sParentStart, sParentEnd);

          SmTArray<SmVertex*> sVertices;
          pFace->GetVertices(sVertices);
          
          // Compare each pFace vertex with parent curve ends
          for (ULONG jj = 0; jj < sVertices.GetSize() && ( !bConnectStart || !bConnectEnd ); jj++ )
            {
              if( (sParentStart.DistanceBetween(sVertices[jj]->GetPoint()) ) < dSamePointTolerance)
                { bConnectStart = TRUE; }
              if( (sParentEnd.DistanceBetween(  sVertices[jj]->GetPoint()) ) < dSamePointTolerance)
                { bConnectEnd = TRUE; }
            } // end loop over vertices

          // A side generated from a corner needs to connect to one vertex, if from a curve it connects to both
          if(   !( bCorner && (bConnectStart || bConnectEnd) )
             && !(!bCorner && (bConnectStart && bConnectEnd) ) )
            { pSurfXSectBrep->DeleteFace(pFace, TRUE); }

        } // end loop over faces

      // Boolean new extruded face to the original Brep
      SmMerge sResultMergeExec(*pContext, pResultBrep, pSurfXSectBrep, dBrepTol);
      sResultMergeExec.ManifoldBoolean(SM_BO_UNION, pResultBrep);

    } // end if bMissingFaces

    


#ifdef SM_DEBUG_CODE     
  if (bDebugMe) 
    {
      SM_ASSERT_VALID(pResultBrep) ;

      SmVector3d sBase(0,0,0) ;
      if (r3DCurves.GetSize() > 0) {
          r3DCurves[0]->EvaluatePoint(r3DCurves[0]->GetNaturalInterval().Evaluate(.5), sBase);
      }

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,1,1) ; if(pResultBrep) pResultBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(2,3, 0,0,1) ; for(di=0;di<r3DCurves.GetSize();di++)
                                    { smgfx_ChangeColor(di!=0) ; r3DCurves[di]->Draw(NULL, TRUE) ; sm_GraphicsLoop() ; }
      smgfx_SetLook(4,5, 0,1,1) ; (dHeight * rCrvsNormal).Draw(&sBase) ; sm_GraphicsLoop() ; 
      sm_GraphicsLoop() ;
     }
#endif // SM_DEBUG_CODE
  
  // Optional End caps added
  // IMPORTANT Cap construction will fail for input curves that are not planar & closed
  SmRegion *pRegion = pResultBrep->GetInfiniteRegion();
  //pResultBrep->m_bEditingEnabled = TRUE;
  SmStatus eStat = SM_SUCCESS;

  // memory note: 1. All faces currently added to the output Brep have been made from CreateFaceFromSurface() from ruled surfaces
  //                  none of these faces reference the bounding taper curves.
  //              2. Adding a Start cap consumes the r3DCurves as the curves for NewEdges made by CreatePlanarFaceWith3DCurves()
  //              3. Adding a End cap consumes the sTrimCrvs=sTrimmedOffsets as the curves for NewEdges
  //                        made by CreatePlanarFaceWith3DCurves()
  //                 3a. Deleting sCompositeCurves does not delete sCurves
  //              4. Output Brep does not end up owning sCurves or sCompositeCurves
  //         So:  1. Don't delete r3DCurves after adding a Start cap
  //              2. Don't delete sTrimCrvs=sTrimmedOffsets after adding an End cap
  //              3. always Delete sCurves and sCompositeCurves - done with SmObjsDelete
  //              4. always Delete sTmpCurves.                  - done with SmObjsDelete
  SmBoolean bCleanupInputCurves  = TRUE;
  SmBoolean bCleanupOffsetCurves = TRUE;

  // when asked - add start endCap
  if (iEndCaps == 1 || iEndCaps == 3)
    {
      SmCurve::CrvsCleanup(r3DCurves,   // i/o: an ordered chain of target curves
                           dBrepTol,    // in : maximum size for a curve to be considered tiny, default:[SM_XSECT_TOL_3D]
                           TRUE);       // in : if True, delete any curves removed from sCrvs
                                        //      default:[FALSE]

      // returns an error and harmlessly fails to add a face to pResultBrep when r3DCurves is not planar and closed.
      SmFace *pFace1 = NULL;
      eStat = pResultBrep->CreatePlanarFaceWith3DCurves(pRegion,   // in : region to contain newFace - not checked
                                                                   //      NULL for infinite region.
                                                        r3DCurves, // in : array of planar curves to bound newFace - curves used by NewEdges
                                                        dBrepTol,  // in : 3d Distance for planarity checks
                                                        pFace1);   // out: new face

      if ( eStat == SM_SUCCESS && pFace1 != NULL )
        { bCleanupInputCurves = FALSE; }
    }

  // when asked - add end endCap
  if ( (iEndCaps == 2 || iEndCaps == 3) && !bEmptyOffsets)
    {
      SmFace *pFace2 = NULL;

      // cast to a SmCurve array
      SmTArray < SmCurve *> sOffs;
      for (ii = 0; ii < sTrimCrvs.GetSize(); ii++)
        {
          SmBSplineCurve *pBSC = (SmBSplineCurve *)sTrimCrvs[ii];
          sOffs.Add(pBSC);
        }

      // Order a curves list into one or more head-tail chain(s) 
      SmTArray <ULONG> sStartChain; 
      SmCurve::FixChainCurvesList(sOffs,       // i/o: curves to order into chains (curve directions possibly reversed, duplicates possibly deleted)
                                  sStartChain, // out: index of each chain start (chain includes curves from this index to next index or end of array)
                                               //      rChainStart.GetSize() == number of chains in r3DCurves.
                                  2.*dZoneTol,    // in : dTol = max distance between matching curve endPoints
                                  TRUE);       // in : TRUE = Delete duplicate curves
                                               //      FALSE= Don't (using duplicate curves may confuse the algorithm)

      // There may be two chains/loops at this point
      for (ii = 0; ii < sStartChain.GetSize(); ii++) {
          SmTArray<SmCurve*> sLoop;
          
          ULONG sIndexStart = sStartChain[ii];
          ULONG sIndexEnd = 0;
          if (ii < sStartChain.GetSize() - 1) {
              sIndexEnd= sStartChain[ii + 1];
          }
          else {
              sIndexEnd = sOffs.GetSize();
          }

          do {
              sLoop.Add(sOffs[sIndexStart]);
              sIndexStart++;
          } while (sIndexStart < sIndexEnd);

          // remove short curves and close resulting gaps from chain list
          SmCurve::CrvsCleanup(sLoop, dBrepTol);

          // returns an error and harmlessly fails to add a face to pResultBrep when r3DCurves is not planar and closed.
          eStat = pResultBrep->CreatePlanarFaceWith3DCurves(pRegion,   // in : region to contain newFace - not checked
                                                                       //      NULL for infinite region.
                                                            sLoop,     // in : array of planar curves to bound newFace - curves used by NewEdges
                                                            dBrepTol,  // in : 3d Distance for planarity checks
                                                            pFace2);   // out: new face

          if (eStat == SM_SUCCESS && pFace2 != NULL)
          {
              bCleanupOffsetCurves = FALSE;
          }
      }
    } // end asked to add EndCap check   
  
#ifdef SM_DEBUG_CODE     
  if (bDebugMe) 
    {
      SM_ASSERT_VALID(pResultBrep) ;

      SmVector3d sBase(0,0,0) ;
      if (r3DCurves.GetSize() > 0) {
          r3DCurves[0]->EvaluatePoint(r3DCurves[0]->GetNaturalInterval().Evaluate(.5), sBase);
      }

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,1,1) ; if(pResultBrep) pResultBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 0,0,1) ; for(di=0;di<r3DCurves.GetSize();di++)
                                    { smgfx_ChangeColor(di!=0) ; r3DCurves[di]->Draw(NULL, TRUE) ; sm_GraphicsLoop() ; }
      smgfx_SetLook(4,5, 0,1,1) ; (dHeight * rCrvsNormal).Draw(&sBase) ; sm_GraphicsLoop() ; 
      sm_GraphicsLoop(); 
      smgfx_Erase();
     }
#endif // SM_DEBUG_CODE

  // clean up the Brep
  // If we call StitchAndOrient, at least one of the curves in r3DCurves is corrupted
  // This will cause a crash when we execute the debug draw again bDebugMe = True (see above)
  pResultBrep->ShrinkGeometry();      
  pResultBrep->StitchAndOrient();   // has coincident geometry - won't pass AssertValid() until all coincident geometry is removed.      

#ifdef SM_DEBUG_CODE     
  if (bDebugMe) 
    {
      SM_ASSERT_VALID(pResultBrep) ;

      SmTArray<SmFace*>   sFaces;
      SmTArray<SmEdge*>   sEdges;
      SmTArray<SmVertex*> sVertices;
      pResultBrep->GetFaces(sFaces);
      pResultBrep->GetEdges(sEdges);
      pResultBrep->GetVertices(sVertices);

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,1,1) ; if(pResultBrep) pResultBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(2,3, 1, 1, 0) ; for(di = 0; di < sFaces.GetSize() ; di++) 
        { smgfx_ChangeColor(di != 0) ; sFaces[di]->Draw(); sm_GraphicsLoop() ; }
      smgfx_SetLook(3,4, 1, 0, 0) ; for(di = 0; di < sEdges.GetSize() ; di++) 
        { smgfx_ChangeColor(di!=0) ; sEdges[di]->Draw(); sm_GraphicsLoop() ; }
      smgfx_SetLook(5,6, 1, 1, 1) ; for(di = 0; di < sVertices.GetSize() ; di++) 
        { smgfx_ChangeColor(di!=0) ; sVertices[di]->Draw(); sm_GraphicsLoop() ; }

      sm_GraphicsLoop(); 
     }
#endif // SM_DEBUG_CODE

  // Delete curves if they haven't been incorporated into the Brep.
  if(bCleanupInputCurves)  { SmObjsDelete< SmCurve* > sClean1( &r3DCurves ); }
  if(bCleanupOffsetCurves) { SmObjsDelete< SmCurve* > sClean2( &sTrimCrvs ); }

  // all done
  return (SM_SUCCESS);

} // end SmPrimitiveCreation::CreateTaperExtrude

/*******************************************************************//**
PURPOSE: This is kind of the equivalent of offsetting and shelling for
     a planar profile.  

NOTES: The input curves should correspond to a planar
     face or faces.  
***********************************************************************/
SmStatus SmPrimitiveCreation::OffsetProfile
 (const SmContext    & crContext,       // in : context for new object construction
  SmTArray<SmCurve*> & r3DCurves,       // in : Must all lie in a plane and form at least
                                        //      one closed loop so we can make a face.
  double               dAppoxTol3d,     // in :
  double               dOffsetDistance, // in : Size of Offset
  ULONG                lOffsetType,     // in : Offset Type  1-LEFT, 2-RIGHT, 3-BOTH 
  SmBoolean            bRoundCorners,   // in : TRUE  = Add rounded corners when expanding a corner
                                        //      FALSE = extend corners
  SmBoolean            bShellResult,    // in : Only works for LEFT or RIGHT offsets
  SmBrep            *& pResult,         // out: New Brep containing Face between r3DCurves and their offsets
  SmTArray<SmBoolean>& rInputOwnershipTransferred) // out: one ownership flag per input curve
{
  // init output
  pResult = NULL ;
  rInputOwnershipTransferred.SetSize(r3DCurves.GetSize());
  rInputOwnershipTransferred.SetAll(FALSE);

  // Make Brep to hold new faces - will become the output
  SmBrep * pBrep = new (crContext) SmBrep();

  // Declared first so it destructs (and frees pBrep) last -- the guards
  // below write back into pBrep on destruction (reverse declaration order).
  SmObjDelete sCleanBrep(pBrep);
  SmTemporaryChangeValue<SmBoolean> sChange(pBrep->m_bEditingEnabled, TRUE);
  SmTemporaryChangeValue<SmBoolean> sTempChangeDB( ((SmContext*) &crContext)->GetDoingBooleanRef(), TRUE );

// Remove Composites
//  pBrep->m_bMakeComposites = TRUE;

// gwc: removed next line - sets pBrep->Tol = crContext::ZoneTol3d
//    pBrep->SetTolerance(SM_APPROX_TO_ZONETOL3D(bAppoxTol3d)) ;

  // locals
  ULONG ii, jj ;
  SmTArray<SmFace*> sNewFaces ;

#ifdef SM_DEBUG_CODE
ULONG     di ;
SmBoolean bDebugMe = FALSE ;
  if(bDebugMe)
    {
      smgfx_Erase();
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 0,1,1) ; for(di=0;di<r3DCurves.GetSize();di++) { if(r3DCurves[di]) r3DCurves[di]->DrawParams() ; sm_GraphicsLoop() ; }
      smgfx_SetLook(4,5, 1,0,1) ; if(pResult) pResult->Draw(TRUE) ; sm_GraphicsLoop() ;
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // Make Planar face in pBrep - cleanup curves, make Plane, check planarity

  // GWC_NEEDS_WORK ONE_TOL_IS_BEING_USED_MULTIPLE_WAYS_IN_THE_FOLLOWING_CALL_STACK__ADD_MORE_TOL_ARGS GWC_LINE ;
  SmTArray<SmObject*> sOwnersBefore;
  for(ULONG iCurve=0; iCurve<r3DCurves.GetSize(); iCurve++)
    {
      sOwnersBefore.Add(r3DCurves[iCurve] ? r3DCurves[iCurve]->GetOwner() : NULL);
    }

  SmStatus ePlanarStat = pBrep->CreatePlanarFacesWith3DCurves
                                (pBrep->GetInfiniteRegion(), // in : region to contain newFace-not checked. NULL=infinite region.
                                 r3DCurves,                  // in : planar curves to bound newFace. Consumed when successful.
                                 dAppoxTol3d,                // in : degen curve length, 3D distance to check for curve planarity = SmTol::GetXSectTol3d(this) ;
                                 sNewFaces);                 // out: array of newFaces

  // CreatePlanarFacesWith3DCurves installs curves one at a time and can fail
  // after only a prefix has transferred. Record each transfer before pBrep's
  // failure cleanup deletes its owned curves.
  for(ULONG iCurve=0; iCurve<r3DCurves.GetSize(); iCurve++)
    {
      if(   ePlanarStat == SM_SUCCESS
         || (   r3DCurves[iCurve] != NULL
             && r3DCurves[iCurve]->GetOwner() != sOwnersBefore[iCurve] ) )
        {
          rInputOwnershipTransferred[iCurve] = TRUE;
        }
    }
  SER(ePlanarStat);

#ifdef SM_DEBUG_CODE
  if(bDebugMe)
    {
      SM_DUMP_AND_ASSERT_VALID(pBrep) ;

      smgfx_Erase();
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 0,1,1) ; for(di=0;di<r3DCurves.GetSize();di++) { if(r3DCurves[di]) r3DCurves[di]->DrawParams() ; sm_GraphicsLoop() ; }
      smgfx_SetLook(1,2, 0,0,0) ; for(di=0;di<sNewFaces.GetSize();di++) { if(sNewFaces[di]) sNewFaces[di]->DrawUV() ; sm_GraphicsLoop() ; }
      smgfx_SetLook(4,5, 1,0,1) ; if(pResult) pResult->Draw(TRUE) ; sm_GraphicsLoop() ;
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // more locals
  SmTArray<ULONG>                 sOrients;
  SmTArray<SmCompositeCurve*>     sCompositeCurves;
  SmTArray<SmCompositeCurve*>     sAllCompositeCurves;
  SmObjsDelete<SmCompositeCurve*> sCleanComp(&sAllCompositeCurves);
  SmVector3d sPlaneZ;

  // for every face - build CompositeCurves for every loop
  for(ii=0; ii<sNewFaces.GetSize(); ii++) 
    {
      SmFace *pFace = sNewFaces[ii];
      SER(pFace->GetSurface()->EvaluateNormal(pFace->GetUVDomain().Evaluate(0.5,0.5),TRUE,TRUE,sPlaneZ));

      // build a CompositeCurve for every loop
      SER(pFace->CreateComposites(crContext,sCompositeCurves));

      // build sOrients array
      for(jj=0; jj<sCompositeCurves.GetSize(); jj++) 
        { sOrients.Add(lOffsetType) ; }

      sAllCompositeCurves.Append(sCompositeCurves);
    }  // end iter every face - building CompositeCurves for every loop

#ifdef SM_DEBUG_CODE
  if(bDebugMe)
    {
      smgfx_Erase();
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 0,1,1) ; for(di=0;di<r3DCurves.GetSize();di++) { if(r3DCurves[di]) r3DCurves[di]->DrawParams() ; sm_GraphicsLoop() ; }
      smgfx_SetLook(1,2, 0,0,0) ; for(di=0;di<sNewFaces.GetSize();di++) { if(sNewFaces[di]) sNewFaces[di]->DrawUV() ; sm_GraphicsLoop() ; }
      smgfx_SetLook(5,6, 1,0,0) ; for(di=0;di<sAllCompositeCurves.GetSize();di++) { if(sAllCompositeCurves[di]) sAllCompositeCurves[di]->DrawParams() ; sm_GraphicsLoop() ; }
      smgfx_SetLook(4,5, 1,0,1) ; if(pResult) pResult->Draw(TRUE) ; sm_GraphicsLoop() ;
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // locals - upcoming CreateOffsetsOfManyCurves() 
  SmTArray<SmCurve*>        sOffsetCurves ;
  SmTArray<SmBSplineCurve*> sBSplineCurves ; 
  SmOffsetCornerType        eCornerType = (bRoundCorners) ? SM_OC_FILLET_CORNER : SM_OC_LINEAR_EXTENSION;
  
  // offset the set of CompositeCurves      // GWC_NEEDS_WORK GET_BETTER_VALUES_FOR_TOLERANCE_IN_FOLLOWING_CALL GWC_LINE ;
  SER(SmCompositeCurve::CreateOffsetsOfManyCurves
        (crContext,                                      // in : context for new object construction
         sAllCompositeCurves,                            // in : Array of composite curves to offset
         sOrients,                                       // in : Corresponding direction of each composite
                                                         //      curve to offset.  1-LEFT, 2-RIGHT, 3-BOTH
         dAppoxTol3d,                                    // in : Minimum distance at which offset curve end-gaps are filled with corner curves
         dAppoxTol3d/4,                                  // in : Tolerance to which BSpline Approximations to exact offset curves are built
         sPlaneZ,                                        // in : Defines, along with the curve's parameter direction,
                                                         //      the right and left hand offset directions.
         eCornerType,                                    // in : SM_OC_LINEAR_EXTENSION: corner = 2 lines from given ends to common linear extension xsect point.
                                                         //      SM_OC_FILLET_CORNER: corner = fillet arc centered on crVertexPoint running to given end points
                                                         //      SM_OC_LINEAR_CHAMFER: corner = line between endpoints (result is actually within offset distance so bTrimResults must = False).
         TRUE,                                           // in : TRUE = concave raw offsets are intersected and trimmed back to common intersection points
                                                         //      FALSE= skip trim step
         dOffsetDistance,                                // in : offset distance (a negative value negates the offset direction)
         sBSplineCurves)) ;                              // out: Resulting offset curves will have attribute attached
                                                         //      describing origination of curve
                                                         // in : TRUE = if BSplineCurve just copy it (preserves CrvParams)
                                                         //      FALSE= approximate BSplineCurves (changes CrvParams), NonBSplineCrvs always Approximated
                                                         //      default:[FALSE]
                                                         // out: optional MaxGap for each output approximation
#ifdef SM_DEBUG_CODE
  if(bDebugMe)
    {
      smgfx_Erase();
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 0,1,1) ; for(di=0;di<r3DCurves.GetSize();di++) { if(r3DCurves[di]) r3DCurves[di]->DrawParams() ; sm_GraphicsLoop() ; }
      smgfx_SetLook(1,2, 0,0,0) ; for(di=0;di<sNewFaces.GetSize();di++) { if(sNewFaces[di]) sNewFaces[di]->DrawUV() ; sm_GraphicsLoop() ; }
      smgfx_SetLook(5,6, 1,0,0) ; for(di=0;di<sAllCompositeCurves.GetSize();di++) { if(sAllCompositeCurves[di]) sAllCompositeCurves[di]->DrawParams() ; sm_GraphicsLoop() ; }
      smgfx_SetLook(5,6, 0,1,0) ; for(di=0;di<sBSplineCurves.GetSize();di++) { if(sBSplineCurves[di]) sBSplineCurves[di]->DrawParams() ; sm_GraphicsLoop() ; }
      smgfx_SetLook(4,5, 1,0,1) ; if(pResult) pResult->Draw(TRUE) ; sm_GraphicsLoop() ;
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // Temp workaround to appease Linux gcc compiler - copy array for type casting - yuk!
  // Original code with typecast has warning: dereferencing type-punned pointer will break strict-aliasing rules [-Wstrict-aliasing]
  for(jj=0;jj<sBSplineCurves.GetSize();jj++) { sOffsetCurves.Add((SmCurve*)sBSplineCurves[jj]); }
  
  // NewBrep to hold Planar faces made from the planar CompositeCurves
  SmExtent3d  sBBox;
  SmBrep    * pNewBrep = new (crContext) SmBrep();
  SmObjDelete sCleanNewBrep(pNewBrep);
  SmTemporaryChangeValue<SmBoolean> sChange2(pNewBrep->m_bEditingEnabled, TRUE);
  pBrep->CalculateBoundingBox(sBBox);

  // double dSize = sBBox.GetSize().Length();
// Remove Composites
//   pNewBrep->m_bMakeComposites = TRUE;
  
  // gwc: removed next line - sets pNewBrep->Tol = crContext::ZoneTol3d
  //    pNewBrep->SetTolerance(SM_APPROX_TO_ZONETOL3D(bAppoxTol3d)) ;
  
  // create a face in the infiniteRegion for every closed loop of curves
  SER(pNewBrep->CreatePlanarFacesWith3DCurves(pNewBrep->GetInfiniteRegion(),   // in : region to contain newFace-not checked. NULL=infinite region. 
                                              sOffsetCurves,                   // in : planar curves to bound newFace. Consumed when successful. 
                                              SmTol::GetXSectTol3d(pNewBrep),  // NotUsed: in : 3D distance to check for curve planarity 
                                              sNewFaces));                     // out: array of newFaces 
#ifdef SM_DEBUG_CODE
  if(bDebugMe)
    {
      SM_DUMP_AND_ASSERT_VALID(pNewBrep) ;

      smgfx_Erase();
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 0,1,1) ; for(di=0;di<r3DCurves.GetSize();di++) { if(r3DCurves[di]) r3DCurves[di]->DrawParams() ; sm_GraphicsLoop() ; }
      smgfx_SetLook(1,2, 0,0,0) ; for(di=0;di<sNewFaces.GetSize();di++) { if(sNewFaces[di]) sNewFaces[di]->DrawUV() ; sm_GraphicsLoop() ; }
      smgfx_SetLook(5,6, 1,0,0) ; for(di=0;di<sAllCompositeCurves.GetSize();di++) { if(sAllCompositeCurves[di]) sAllCompositeCurves[di]->DrawParams() ; sm_GraphicsLoop() ; }
      smgfx_SetLook(5,6, 0,1,0) ; for(di=0;di<sBSplineCurves.GetSize();di++) { if(sBSplineCurves[di]) sBSplineCurves[di]->DrawParams() ; sm_GraphicsLoop() ; }
      smgfx_SetLook(4,5, 1,0,1) ; if(pResult) pResult->Draw(TRUE) ; sm_GraphicsLoop() ;
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // when asked Shell result - Inset of faces branch
  if (bShellResult && lOffsetType == 1) 
    { 
      // 2D Boolean: Result = pBrep - pNewBrep  (note: increments unlocked mark value)
      SmStatus eBooleanStat = SmPrimitiveCreation::Boolean2D(pBrep,            // in : A of A operator B, returned as rpResult
                                                             pNewBrep,         // in : B of A operator B, deleted by this operation
                                                             SM_2D_DIFFERENCE, // in : oneof: SM_2D_UNION,        = Union of 2D regions A and B
                                                                               //             SM_2D_INTERSECTION, = Intersection of 2D regions A and B
                                                                               //             SM_2D_DIFFERENCE,   = Difference - A minus B
                                                                               //             SM_2D_EXCLUSIVE_OR, = XOR = (A union B) - (A intersect B)
                                                                               //             SM_2D_MERGE         = Merge Operation
                                                             pResult);         // out: Modified pBrepA
      SER(eBooleanStat);
      NER(pResult);

      // Boolean2D consumed pNewBrep and returned pBrep. Disarm guards only
      // after success so failures restore state and delete both temporaries.
      sChange2.Clear();
      sCleanBrep.Clear();
      sCleanNewBrep.Clear();
    } // end Shelling a Left offset branch - Inset of faces
  
  // when asked Shell result - Offset of faces branch
  else if (bShellResult && lOffsetType == 2) 
    { 
      // 2D Boolean: Result = pNewBrep - pBrep  (note: increments unlocked mark value)
      SmStatus eBooleanStat = SmPrimitiveCreation::Boolean2D(pNewBrep,         // in : A of A operator B, returned as rpResult
                                                             pBrep,            // in : B of A operator B, deleted by this operation
                                                             SM_2D_DIFFERENCE, // in : oneof: SM_2D_UNION,        = Union of 2D regions A and B
                                                                               //             SM_2D_INTERSECTION, = Intersection of 2D regions A and B
                                                                               //             SM_2D_DIFFERENCE,   = Difference - A minus B
                                                                               //             SM_2D_EXCLUSIVE_OR, = XOR = (A union B) - (A intersect B)
                                                                               //             SM_2D_MERGE         = Merge Operation
                                                             pResult);         // out: Modified pBrepA
      SER(eBooleanStat);
      NER(pResult);

      // Boolean2D consumed pBrep and returned pNewBrep. Disarm guards only
      // after success so failures restore state and delete both temporaries.
      sChange.Clear();
      sCleanBrep.Clear();
      sCleanNewBrep.Clear();
    } // end Shelling a Right offset branch - outset of faces
  else // no shelling branch
    {
      sCleanNewBrep.Clear();
      pResult = pNewBrep;
    } // end no shelling branch
  
#ifdef SM_DEBUG_CODE
  if(bDebugMe)
    {
      SM_DUMP_AND_ASSERT_VALID(pResult) ;

      smgfx_Erase();
      smgfx_SetLook(3,4, 0,1,1) ; for(di=0;di<r3DCurves.GetSize();di++) { if(r3DCurves[di]) r3DCurves[di]->DrawParams() ; sm_GraphicsLoop() ; }
      // smgfx_SetLook(5,6, 1,0,0) ; for(di=0;di<sAllCompositeCurves.GetSize();di++) { if(sAllCompositeCurves[di]) sAllCompositeCurves[di]->DrawParams() ; sm_GraphicsLoop() ; }
      // smgfx_SetLook(5,6, 0,1,0) ; for(di=0;di<sBSplineCurves.GetSize();di++) { if(sBSplineCurves[di]) sBSplineCurves[di]->DrawParams() ; sm_GraphicsLoop() ; }
      smgfx_SetLook(4,5, 1,0,1) ; if(pResult) pResult->Draw(TRUE) ; sm_GraphicsLoop() ;
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // all done
  return SM_SUCCESS;

} // end SmPrimitiveCreation::OffsetProfile

/*******************************************************************//**
PURPOSE: Create solid primitive by sweeping a closed profile along a path.
    This function calls CreateCurveSweep() with a list of curves

NOTES:
     Please see notes for SmPrimitiveCreation::CreateCurveSweep

***********************************************************************/
SmStatus SmPrimitiveCreation::CreateCurveSweepFromEdges
 (SmTArray<SmEdge*> & rEdges,          // in : curves to sweep
  SmBSplineCurve    * pPathCurve,      // in : path curve along which to sweep
  SmBSplineCurve    * pScaleReference, // in : used together with pScaleCurve to define change along path
  SmBSplineCurve    * pScaleCurve,     // in : Defines how the profiles can change as it travels down path
  double              bAppoxTol3d,     // in : 3D distance tolerance
  SmSweepOptions    * pOptions,        // in : several independent boolean flags
  SmTArray<SmFace*> & rStartFaces,     // out: list of new faces for start cap
  SmTArray<SmFace*> & rSideFaces,      // out: list of new faces for sweep surface
  SmTArray<SmFace*> & rEndFaces )      // out: list of new faces for end cap
{
    // Get Profile curves from input edges
    const SmContext * pContext = m_pRegion->GetContext();
    SmTArray <SmCurve*> sCurves;
    SmObjsDelete<SmCurve*> sClean( &sCurves );

    for( ULONG ii = 0; ii < rEdges.GetSize(); ii++ ) {
        SmEdge  * pEdge = rEdges[ii];
        SmCurve * pCrv = NULL;
        pEdge->GetCurve()->Copy( *pContext, pCrv );
        SmExtent1d sInterval = pEdge->GetInterval();
        pCrv->Trim( sInterval, FALSE );
        sCurves.Add( pCrv );
    }

    // Now call Sweep with profile curves
    SER( CreateCurveSweep( sCurves, pPathCurve, pScaleReference, pScaleCurve, bAppoxTol3d,
        pOptions, rStartFaces, rSideFaces, rEndFaces ));

    return SM_SUCCESS;
} // end SmPrimitiveCreation::CreateCurveSweepFromEdges

/*******************************************************************//**
PURPOSE: Create a solid primitive by sweeping the given faces or edges.

NOTES: 
    Input faces and their curves are borrowed. Working curve copies are passed
    to CreateCurveSweep, so the inputs remain unchanged.
    Please see notes for SmPrimitiveCreation::CreateCurveSweep

***********************************************************************/
SmStatus SmPrimitiveCreation::CreateCurveSweepFromFaces
 (SmTArray<SmFace*> & rFaces,           // in : faces to sweep
  SmBSplineCurve    * pPathCurve,       // in : path curve along which to sweep
  SmBSplineCurve    * pScaleReference,  // in : used together with pScaleCurve to define change along path
  SmBSplineCurve    * pScaleCurve,      // in : Defines how the profiles can change as it travels down path
  double              dAppoxTol3d,      // in : 3D distance tolerance
  SmSweepOptions    * pOptions,         // in : several independent boolean flags
  SmTArray<SmFace*> & rStartFaces,      // out: list of new faces for start cap
  SmTArray<SmFace*> & rSideFaces,       // out: list of new faces added to this brep
  SmTArray<SmFace*> & rEndFaces)        // out: list of new faces for end cap
{
    // Get Profile curves from input faces
    const SmContext  * cpContext = m_pRegion->GetContext();
    SmTArray<SmCurve*> sCurves;
    SmTArray<SmEdge*>  sEdges;
    SmObjsDelete<SmCurve*> sClean( &sCurves );
    ULONG ii, jj;
    for( ii = 0; ii < rFaces.GetSize(); ii++ ) {
        rFaces[ii]->GetEdges( sEdges );
        for( jj = 0; jj < sEdges.GetSize(); jj++ ) {
            SmEdge *pEdge = sEdges[jj];
            SmCurve *pCrv = NULL;
            pEdge->GetCurve()->Copy( *cpContext, pCrv );
            SmExtent1d sInterval = pEdge->GetInterval();
            pCrv->Trim( sInterval, FALSE );

            sCurves.Add( pCrv );
        }
    }

    // Now call Sweep with profile curves
    SER( CreateCurveSweep( sCurves, pPathCurve, pScaleReference, pScaleCurve, dAppoxTol3d,
        pOptions, rStartFaces, rSideFaces, rEndFaces ));




    return SM_SUCCESS;

} // end SmPrimitiveCreation::CreateCurveSweepFromFaces

/*******************************************************************//**
PURPOSE: Create a solid primitive by sweeping the given curves along the given path.

NOTES: 

   Input curves are borrowed. The profile, path, and optional scale curves are
   copied before modification, so the inputs remain unchanged.

   bMoveProfileCenterToPath will align a copy of the profiles so that their
   combined center will travel down the path. Otherwise, start point will used.

   bOrientProfilePerpendicularToPath will align a copy of the profiles so that
   they are orthogonal to the path's initial direction.

   bTranslationalSweep will not reorient the profiles as the sweep the path

   bMoveToProfile will move the result to the profile curves

   bCapEnds will optionally add faces to each end cap.

   The scaled curve is defined as the distance from itself to the 
   path curve.  The scaled curve only works when bTranslationalSweep is FALSE.

***********************************************************************/
SmStatus SmPrimitiveCreation::CreateCurveSweep
( SmTArray<SmCurve*> & rProfileCurves,   // in : source curves to sweep
 SmBSplineCurve     * pPathCurve,       // in : path curve along which to sweep
 SmBSplineCurve     * pScaleReference,  // in : used together with pScaleCurve to define change along path
 SmBSplineCurve     * pScaleCurve,      // in : Defines how the profiles can change as it travels down path
 double               bAppoxTol3d,      // in : 3D distance tolerance
 SmSweepOptions     * pOptions,         // in : several independent boolean flags
 SmTArray<SmFace*>  & rStartFaces,      // out: list of new faces for start cap
 SmTArray<SmFace*>  & rSideFaces,       // out: list of new faces added to this brep
 SmTArray<SmFace*>  & rEndFaces,        // out: list of new faces for end cap
 SmBoolean            bTestContinuity )  // in : if true test the continuity of the profile curves
{
  // empty entity maps - m_vMapToSame, m_vMapToHigher, m_vMapFromSame, and m_vMapFromLower
  InitEntityMaps();

  rSideFaces.ReSet();
  ULONG lNumCurves = rProfileCurves.GetSize();

  if(lNumCurves < 1) { SER( SM_ERR ); }

  // Locals
  ULONG ii, jj;
  SmBrep *pBrep = m_pRegion->GetBrep();
  const SmContext *pContext = pBrep->GetContext();
  SmBSplineCurve *pPathCurveCopy = new (pContext) SmBSplineCurve( *pPathCurve );
  SmObjDelete sCleanPath( pPathCurveCopy );
  SmTemporaryChangeValue <SmBoolean> sTempChangeDB( ((SmContext*) pContext)->GetDoingBooleanRef(), TRUE );

  // Copy each input curve as an SmBSplineCurve,
  // and collect the copies in sNurbs array.
  // Note, if any input curve is not an SmBSplineCurve,
  // it will be ignored.
  SmTArray<SmBSplineCurve*> sNurbs;
  SmObjsDelete<SmBSplineCurve*> sCleanNurbs( &sNurbs );
  for(ii = 0; ii < lNumCurves; ii++)
  {
    SmBSplineCurve *pBSC = SM_CAST_PTR( SmBSplineCurve, rProfileCurves[ii] );
    if(!pBSC) { continue; }

    pBSC = new (*pContext) SmBSplineCurve( *pBSC );
    sNurbs.Add( pBSC );
  }

  if(bTestContinuity)
  {
    // look for c1 discontinuities and replace with multiple curves
    SmTArray <SmBSplineCurve *>c3DBSPLC1;
    for(ii = 0; ii < sNurbs.GetSize(); ii++)
    {
      sNurbs[ii]->SubdivideAtDiscontinuity
        ( *pContext,  // in : context for new object construction
          SM_CT_C1,   // in : curve is split at internal continuities <= to this value
          c3DBSPLC1 ); // out: newly copied curves - may be 1 curve copy when no internal discontinuities
      if(c3DBSPLC1.GetSize() > 1)
      {
        delete sNurbs[ii]; // delete this curve, which is now replaced
        sNurbs.RemoveAt( ii );
        for(jj = 0; jj < c3DBSPLC1.GetSize(); jj++)
        { sNurbs.InsertAt( ii++, c3DBSPLC1[jj] ); }
        ii--;
      }
      else if(c3DBSPLC1.GetSize() == 1)
      {
        delete c3DBSPLC1.GetAt( 0 );
      }
    }
  }

  // reset lNumCurves now that they've been split
  lNumCurves = sNurbs.GetSize();

  // Deal with scaling.
  SmBSplineCurve *pScaled = NULL;
  SmObjDelete sCleanScaled;
  if(pScaleCurve && pScaleReference)
  {
    SmBSplineCurve *pScaleCurveCopy = new (*pContext) SmBSplineCurve( *pScaleCurve );
    SmObjDelete sCleanScaleCurve( pScaleCurveCopy );

    // Reparameterize to the same as the path curve.
    SmExtent1d sIvl = pPathCurveCopy->GetNaturalInterval();
    SER( pScaleCurveCopy->EditParameterization( sIvl ) );
    SmTArray<SmPoint3d> sPoints;
    ULONG lMaxKnots = smos_Max( pPathCurveCopy->GetNumberOfUniqueKnots(),
        pScaleCurveCopy->GetNumberOfUniqueKnots() );
    ULONG lNumPnts = smos_Max( 30, lMaxKnots * 10 );
    SmPoint3d sStartSC, sStartP;
    SER( pScaleCurveCopy->EvaluatePoint( sIvl.GetMin(), sStartSC ) );
    SmExtent1d sSRIvl = pScaleReference->GetNaturalInterval();
    SmSolution sSData[16];
    SmSolutionArray sSolutions( 16, sSData );
    SER( pScaleReference->GlobalPointSolve( sSRIvl, SM_SO_MINIMIZE, sStartSC, SM_EFF_ZERO_SQRT, NULL, NULL,
         SM_SR_SINGLE, sSolutions ) );
    SER( pScaleReference->EvaluatePoint( sSolutions[0].m_vStart[0], sStartP ) );
    double dScale = 1.0 / sStartSC.DistanceBetween( sStartP );
    SmTArray<double> sParams;
    for(ii = 0; ii <= lNumPnts; ii++)
    {
      double dT = (ii*1.0) / lNumPnts;
      double dParam = sIvl.Evaluate( dT );
      sParams.Add( dParam );
      SER( pScaleCurveCopy->EvaluatePoint( dParam, sStartSC ) );
      SER( pScaleReference->GlobalPointSolve( sSRIvl, SM_SO_MINIMIZE, sStartSC, SM_EFF_ZERO_SQRT, NULL, NULL,
           SM_SR_SINGLE, sSolutions ) );
      SER( pScaleReference->EvaluatePoint( sSolutions[0].m_vStart[0], sStartP ) );
      double dDist = sStartSC.DistanceBetween( sStartP );
      dDist = dDist * dScale;
      sPoints.Add( SmPoint3d( dDist, dDist, dDist ) );
    }
    // double dTol = bAppoxTol3d / 10.0;
    SmStatus eInterpolateStat = SmBSplineCurve::InterpolatePoints(
        *pContext, sPoints, &sParams, 3, NULL, NULL, FALSE, SM_IT_CHORDLENGTH, pScaled );
    sCleanScaled.SetObj( pScaled );
    SER( eInterpolateStat );
    NER( pScaled );
    SER( pScaled->EditParameterization( sIvl ) );
    SmTArray<SmBSplineCurve*> sCurves;
    sCurves.Add( pPathCurveCopy );
    sCurves.Add( pScaled );
    SER( SmBSplineCurve::MakeCurvesCompatible( sCurves ) );

  } // end if doing scaling.

  SmVector3d sOrigin( 0, 0, 0 ), sXAxis( 1, 0, 0 ), sYAxis( 0, 1, 0 ), sZAxis( 0, 0, 1 );

#ifdef SM_DEBUG_CODE
  // BLACK: Display axes and input curves
  SmBoolean bDebugMe = FALSE;
  if(bDebugMe)
  {
    smgfx_Erase();
    SmBSplineCurve* line = NULL;
    SmBSplineCurve::CreateLineSegment( *pContext, 3, sOrigin, sXAxis * 5, line );
    smgfx_SetLook( 1, 2, 0, 0, 0 ); line->Draw();  sm_GraphicsLoop();
    SmBSplineCurve::CreateLineSegment( *pContext, 3, sOrigin, sYAxis * 10, line );
    smgfx_SetLook( 1, 2, 0, 0, 0 ); line->Draw();  sm_GraphicsLoop();
    SmBSplineCurve::CreateLineSegment( *pContext, 3, sOrigin, sZAxis * 20, line );
    smgfx_SetLook( 1, 2, 0, 0, 0 ); line->Draw();  sm_GraphicsLoop();
    smgfx_SetLook( 1, 2, 0, 0, 0 ); pPathCurveCopy->Draw(); sm_GraphicsLoop();
    for(ii = 0; ii < sNurbs.GetSize(); ii++)
    {
      smgfx_SetLook( 1, 2, 0, 0, 0 );
      sNurbs[ii]->Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
  }
#endif // SM_DEBUG_CODE

  // Move Path to orgin
  SmVector3d sPathPtAndDir[2];
  SER( pPathCurveCopy->Evaluate( pPathCurveCopy->GetNaturalInterval().GetMin(), 1, TRUE, sPathPtAndDir ) );
  SmAxis2Placement sPathToOrigin( sOrigin, sXAxis, sYAxis );
  sPathToOrigin.Translate( -sPathPtAndDir[0] );
  pPathCurveCopy->Transform( sPathToOrigin );

  // Move Profiles to orgin
  SmVector3d sProfileRefPt( 0, 0, 0 );
  if(pOptions->bMoveProfileCenterToPath)
  {
    // If moving profile to center of path then we need to 
    // compute the centroid of all of the edges.
    SmExtent3d sBBoxTotal, sBBox;
    for(ii = 0; ii < lNumCurves; ii++)
    {
      SmCurve* pCrv = sNurbs[ii];
      pCrv->CalculateBoundingBox( pCrv->GetNaturalInterval(), &sBBox );
      sBBoxTotal.AddPoint3d( sBBox.GetMin() );
      sBBoxTotal.AddPoint3d( sBBox.GetMax() );
    }
    sProfileRefPt = sBBoxTotal.GetMid();
  }
  else
  {
    sNurbs[0]->EvaluatePoint( sNurbs[0]->GetNaturalInterval().GetMin(), sProfileRefPt );
  }

  // Translate Profiles to Origin
  SmAxis2Placement sProfileToOrigin( sOrigin, sXAxis, sYAxis );
  sProfileToOrigin.Translate( -sProfileRefPt );
  for(ii = 0; ii < lNumCurves; ii++)
  {
    sNurbs[ii]->Transform( sProfileToOrigin );
  }

#ifdef SM_DEBUG_CODE
  // RED: Path is oriented toward x axis and profiles have the same input relationship
  if(bDebugMe)
  {
    smgfx_SetLook( 1, 2, 1, 0, 0 ); pPathCurveCopy->Draw(); sm_GraphicsLoop();
    for(ii = 0; ii < sNurbs.GetSize(); ii++)
    {
      sNurbs[ii]->Draw();
      sm_GraphicsLoop();
    }
    // draw profile reference point
    smgfx_SetLook( 3, 4, 1, 0, 0 ); sProfileRefPt.Draw(); sm_GraphicsLoop();
    sm_GraphicsLoop();
    sm_GraphicsLoop();
  }
#endif // SM_DEBUG_CODE

  // Create matrix that orients path so start tangent in X Direction.
  SER( pPathCurveCopy->Evaluate( pPathCurveCopy->GetNaturalInterval().GetMin(), 1, TRUE, sPathPtAndDir ) );

  SmVector3d sFromX, sFromY, sFromZ;
  sPathPtAndDir[1].MakeUnitOrthoVectors( NULL, sFromX, sFromY, sFromZ );

  SmVector3d sToXAxis( 1, 0, 0 ), sToYAxis( 0, 1, 0 );
  SmAxis2Placement sPathRot( sOrigin, sFromX, sFromY,
                             sOrigin, sToXAxis, sToYAxis );

  pPathCurveCopy->Transform( sPathRot );

  // Maintain relationship between path and profile by transforming profiles too
  for(ii = 0; ii < lNumCurves; ii++)
  {
    sNurbs[ii]->Transform( sPathRot );
  }

#ifdef SM_DEBUG_CODE
  // BLUE: Path is oriented toward x axis and profiles have the same input relationship
  if(bDebugMe)
  {
    smgfx_SetLook( 1, 2, 0, 0, 1 ); pPathCurveCopy->Draw(); sm_GraphicsLoop();
    for(ii = 0; ii < lNumCurves; ii++)
    {
      sNurbs[ii]->Draw();
      sm_GraphicsLoop();
    }
    sm_GraphicsLoop();
  }
#endif // SM_DEBUG_CODE

  // We will use these again to move the resulting sweep back to either
  // the original path or the profile location
  SmAxis2Placement sPathRotBack;
  sPathRot.Invert( sPathRotBack );

  SmAxis2Placement sPathToOriginBack;
  sPathToOrigin.Invert( sPathToOriginBack );

  SmAxis2Placement sProfileToOriginBack;
  sProfileToOrigin.Invert( sProfileToOriginBack );

  // Now orient the profile orthogonal to the path
  SmAxis2Placement sProfileTrans( sOrigin, sXAxis, sYAxis );

#ifdef SM_DEBUG_CODE
  // Green Path is oriented toward x axis and blue profiles have the same input relationship
  if(bDebugMe)
  {
    smgfx_SetLook( 1, 2, 0, 1, 0 ); pPathCurveCopy->Draw(); sm_GraphicsLoop();
    smgfx_SetLook( 1, 2, 0, 0, 1 );
    for(ii = 0; ii < lNumCurves; ii++)
    {
      sNurbs[ii]->Draw();
      sm_GraphicsLoop();
    }
    sm_GraphicsLoop();
  }
#endif // SM_DEBUG_CODE

  if(pOptions->bOrientProfilePerpendicularToPath)
  {
    SmVector3d xAxis( 1, 0, 0 );

    // If orienting profile then we need the profile's normal
    // cast to SmCurves for ComputeCrvsNormal
    SmVector3d sProfileNormal;
    SmTArray<SmCurve*> sCurves(lNumCurves, NULL, lNumCurves);
    for ( ii = 0 ; ii < lNumCurves; ii++ )
    { sCurves[ii] = SM_CAST_PTR( SmCurve, sNurbs[ii] ); }

    if(SmCurve::ComputeCrvsNormal( sCurves, sProfileNormal ) == SM_ERR)
    {
      SmVector3d sStartVector[2];
      SmCurve* pProfile = sNurbs[0];
      SmExtent1d sDomain = pProfile->GetNaturalInterval();
      pProfile->Evaluate( sDomain.Evaluate( 0 ), 1, TRUE, sStartVector );
      // Because begin tangent of path should be x axis at this point

      sProfileNormal = xAxis * sStartVector[1];
    }

    SmTArray<SmCurve*>sPathCurves;
    sPathCurves.Add( pPathCurveCopy );

    SmVector3d sPathNormal;
    SmCurve::ComputeCrvsNormal( sPathCurves, sPathNormal );

    // determine the relationship between profileNormal and the xAxis
    // if profileNormal is neg X at all then flip it

    double dot = sProfileNormal.Dot( xAxis );
    if(dot < 0)
    {
      sProfileNormal = -sProfileNormal;
    }

    // determine the relationship between profileNormal and the pathNormal
    // if in the same plane, then must rotate
    //double dAngleBetweenProfileAndPath;
    //sPathNormal.AngleBetween(sProfileNormal, dAngleBetweenProfileAndPath);

    //dot = sPathNormal.Dot(sProfileNormal);

    // Determine angle to transform profile curves
    double angle = 0.0;
    //if (dot != 0.0) {
    xAxis.AngleBetween( sProfileNormal, angle );
    //}

    // Shade3d0316_60    requires that dot != 0
    // xAxis.AngleBetween(sProfileNormal, angle);
    // angle is recomputed to 1.57

    // Shade3d0428_70    does not work
    // sProfileNormal (0.7, 0.7, 0) ** 
    // sPathNormal (0,0,1)
    // dAngleBetweenProfileAndPath = 1.57
    // dot = 0.0
    // xAxis.AngleBetween(sProfileNormal, angle);
    // angle is recomputed to 0.7894

    // Shade3d0316_B
    // sProfileNormal (0,0,1)
    // sPathNormal (0,-1,0)
    // dAngleBetweenProfileAndPath = 1.57
    // dot = 0.0
    // NO xAxis.AngleBetween(sProfileNormal, angle);
    // angle = 0.0 then it works

    // This is where we transform the profile curves
    SmVector3d rotationAxis = xAxis * sProfileNormal;
    sProfileTrans.RotateAboutAxis( -angle, rotationAxis );

    // Transform Profile curves according to input options
    for(ii = 0; ii < lNumCurves; ii++)
    {
      sNurbs[ii]->Transform( sProfileTrans );
    }
  }

#ifdef SM_DEBUG_CODE
  // GREEN: Move profile curves according to input parameters
  if(bDebugMe)
  {
    smgfx_SetLook( 1, 2, 0, 1, 0 );
    for(ii = 0; ii < lNumCurves; ii++)
    {
      sNurbs[ii]->Draw();
      sm_GraphicsLoop();
    }
    sm_GraphicsLoop();
  }
#endif // SM_DEBUG_CODE

  SmTArray<SmCompositeCurve*> sComposites;
  SmObjsDelete<SmCompositeCurve*> sCleanComposites( &sComposites );

  // Now create the composite curve(s) from the curves in sNurbs.
  double dTol = pBrep->GetTolerance();
  SER( SmCompositeCurve::BuildCompositesFromCurves
  ( *pContext,       // in : context for new object construction 
       sNurbs,          // in : unordered curves to connect into composites                             
                        //      Included curve ownership moves to the output composites. Tiny omitted
                        //      curves remain caller-owned.
       TRUE,            // in : (not yet used) bMakeCurvesHomogeneous : TRUE = Approx curves with deg 3 NUBs 
       0.0,             // in : (not yet used) dThisApproxTol3d = tol used if an approximation is required. 
       dTol,            // in : if      gap < dSamePointTolerance, connect 
       0.0,             // in : (not yet used) else if gap < dOptDistanceToAverage,    snap to avg and connect - not implemented 
       0.0,             // in : (not yet used) else if gap < dOptDistanceToExtendTrim, trim and connect        - not implemented 
       0.0,             // in : else if gap < dOptDistanceToCreateLine, insert line seg and connect 
       0.0,             // in : (not yet used) else if gap < dOptDistanceToCreateBlend, insert blend and connect 
       sComposites ) );  // out: Composite curves own included input curves and any new gap curves.

  // Stop the local guard from deleting curves whose ownership moved to a
  // composite. It will still delete any tiny curves omitted by the builder.
  for(ii = 0; ii < sComposites.GetSize(); ii++)
  {
    ULONG lDimension = 0;
    SmBoolean bClosed = FALSE;
    SmTArray<SmCurve*> sCompositeCurves;
    SmStatus eCanonicalStat = sComposites[ii]->GetCanonical(
        lDimension, sCompositeCurves, bClosed );
    if(eCanonicalStat != SM_SUCCESS)
    {
      // The composites already own some of the pointers. Prefer a leak on this
      // unexpected path to deleting an object through two owners.
      sCleanNurbs.Clear();
      SER( eCanonicalStat );
    }
    for(jj = 0; jj < sCompositeCurves.GetSize(); jj++)
    {
      for(ULONG kk = 0; kk < sNurbs.GetSize(); kk++)
      {
        if(sNurbs[kk] == sCompositeCurves[jj])
        {
          sNurbs.SetAt( kk, NULL );
          break;
        }
      }
    }
  }
// Sweep each profile in sComposites along path.
// Transform each profile according to input parameters to align with path
// Then move resulting surface back to original path position
  ULONG lNumComposites = sComposites.GetSize();
  for(ii = 0; ii < lNumComposites; ii++)
  {
    SmCompositeCurve *pComp = sComposites[ii];
    SmBSplineCurve *pCompNurb = NULL;
    SER( pComp->MakeCompositeNurb( *pContext, pCompNurb ) );

    // Sweep the profile along the path
    // The path has already been rotated along the x axis
    SmObjDelete sCleanNurb( pCompNurb );
    SmBSplineSurface *pNewBSS = NULL;
    SER( SmBSplineSurface::CreateSweepSurface( *pContext, pCompNurb, pPathCurveCopy , pScaled,
         pOptions->bTranslationalSweep, bAppoxTol3d, pNewBSS ) );

#ifdef SM_DEBUG_CODE
    // GREEN: Draw each swept surface
    if(bDebugMe)
    {
      pNewBSS->Dump();
      smgfx_SetLook( 1, 2, 0, 1, 0 ); pNewBSS->Draw();    sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

    // For now subdivide at G1 continuities to make things
    // which are more filletable.
    SmTArray<SmFace*> sNewFaces;
    pBrep->CreateFacesFromSurface( pNewBSS, pNewBSS->GetNaturalUVDomain(), SM_CT_G1, sNewFaces );
    rSideFaces.Append( sNewFaces );

  } // end for each curve in sComposites

  // Move the resulting surface back to original path position
  pBrep->Transform( sPathRotBack );

#ifdef SM_DEBUG_CODE
  // BLACK: Draw Resulting brep
  if(bDebugMe)
  {
    smgfx_SetLook( 1, 2, 0, 0, 0 ); pBrep->Draw(); sm_GraphicsLoop();
    sm_GraphicsLoop();
  }
#endif // SM_DEBUG_CODE

  // Move the result from the path location to the profile location
  if(pOptions->bMoveToProfile == TRUE)
  {
    pBrep->Transform( sProfileToOriginBack );
  }
  else
  {
    pBrep->Transform( sPathToOriginBack );
  }

#ifdef SM_DEBUG_CODE
  // BLACK: Draw Resulting brep
  if(bDebugMe)
  {
    smgfx_SetLook( 1, 2, 0, 0, 0 ); pBrep->Draw(); sm_GraphicsLoop();
    sm_GraphicsLoop();
  }
#endif // SM_DEBUG_CODE

  // Assemble curves with which to create the start and end caps (top face or faces)
  // Make sure we use the first number of curves in the first cap
  // and the last number of curves in the end cap
  if(pOptions->bCapEndsArg)
  {
    SmSurface* pSurface = NULL;
    SmBSplineCurve* pCrv = NULL;

    //const SmContext *pContext = pBrep->GetContext();
    SmTArray<SmCurve*> sStartCurves, sEndCurves;

    SmObjsDelete<SmCurve*> sStartCleanup(&sStartCurves), sEndCleanup(&sEndCurves);
    SmStatus eCapStat = SM_SUCCESS;

    SmExtent2d sDomain;
    SmSurfParamType eSurfParam = SM_SP_U;
    double dParam;

    ULONG lNumSideFaces = rSideFaces.GetSize();
    for(ii = 0; ii < lNumSideFaces; ii++)
    {
      if(ii < lNumCurves)
      {
        pSurface = rSideFaces[ii]->GetSurface();
        sDomain = pSurface->GetNaturalUVDomain();
        eSurfParam = SM_SP_U;
        dParam = sDomain.GetMin().x;

        if(pOptions->bTranslationalSweep)
        {
          eSurfParam = SM_SP_V;
          dParam = sDomain.GetMin().y;
        }

        eCapStat = pSurface->CreateIsoParametricCurve( *pContext, eSurfParam, dParam, bAppoxTol3d / 10.0, pCrv );
        SER( eCapStat );

#ifdef SM_DEBUG_CODE
        if(bDebugMe)
        {
          smgfx_Erase();
          smgfx_SetLook( 1, 2, 0, 0, 1 );
          pSurface->DrawUV(); sm_GraphicsLoop();

          smgfx_SetLook( 2, 3, 1, 0, 0 );
          pCrv->Draw(); sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE 
        sStartCurves.Add( pCrv );
      }

      // Always get sEndCurves when lNumSideFaces < lNumCurves. In this case lNumSideFaces - lNumCurves 
      // should be a negative number and all is well. But they are ULONGs. So the result is not negative
      // So we end up with no sEndCurves.
      if(lNumCurves > lNumSideFaces || ii >= lNumSideFaces - lNumCurves)
      {
        pSurface = rSideFaces[ii]->GetSurface();
        sDomain = pSurface->GetNaturalUVDomain();
        dParam = sDomain.GetMax().x;
        if(pOptions->bTranslationalSweep)
        {
          eSurfParam = SM_SP_V;
          dParam = sDomain.GetMax().y;
        }

        eCapStat = pSurface->CreateIsoParametricCurve( *pContext, eSurfParam, dParam,  bAppoxTol3d / 10.0, pCrv );
        SER( eCapStat );

#ifdef SM_DEBUG_CODE
        if(bDebugMe)
        {
          smgfx_Erase();
          smgfx_SetLook( 1, 2, 0, 0, 1 );
          pSurface->DrawUV(); sm_GraphicsLoop();

          smgfx_SetLook( 2, 3, 1, 0, 0 );
          pCrv->Draw(); sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE 
        sEndCurves.Add( pCrv );
      }
    } // end for each side face

    // create start and end caps
    eCapStat = pBrep->CreatePlanarFacesWith3DCurves( m_pRegion, sStartCurves, bAppoxTol3d, rStartFaces );
    // A failed cap may already own some curves. Leave those to the Brep;
    // the guards delete only curves that were not transferred to topology.
    for( ULONG kk = 0; kk < sStartCurves.GetSize(); ++kk )
      if( sStartCurves[kk]->GetOwner() != NULL ) { sStartCurves[kk] = NULL; }
    SER( eCapStat );
    sStartCleanup.Clear();
    eCapStat = pBrep->CreatePlanarFacesWith3DCurves( m_pRegion, sEndCurves,   bAppoxTol3d, rEndFaces   );
    for( ULONG kk = 0; kk < sEndCurves.GetSize(); ++kk )
      if( sEndCurves[kk]->GetOwner() != NULL ) { sEndCurves[kk] = NULL; }
    SER( eCapStat );
    sEndCleanup.Clear();


#ifdef SM_DEBUG_CODE
    if(bDebugMe)
    {
      smgfx_SetLook( 3, 4, 0, 0, 0 );
      rStartFaces[0]->DrawUV(); sm_GraphicsLoop();
      rEndFaces[0]->DrawUV(); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE 

  } // end if doing end capping

  pBrep->StitchAndOrient();

  return SM_SUCCESS;

} // end SmPrimitiveCreation::CreateCurveSweep

/*******************************************************************//**
PURPOSE: Copy each segment of the path and extend it by given distance

NOTES: Helper function for CreateSweepAlongPlanarPath
***********************************************************************/
static void ExtendPathCurves
( 
  const SmContext& crContext, 
  SmTArray<SmCurve*>& rPathCurves,
  SmBoolean bIsPathClosed, 
  SmBoolean bIsPathPeriodic, 
  double dDistance,
  SmTArray<SmCurve*>& rsExtendedPathCurves 
)
{
  SmBSplineCurve* pPath = NULL;
  SmBSplineCurve* pPathBefore = NULL;
  SmBSplineCurve* pPathAfter = NULL;

  SmVector3d sPVBefore[2];
  SmVector3d sPVAfter [2];
  ULONG ii;

  for ( ii = 0; ii < rPathCurves.GetSize(); ii++ )
  {
    pPath = (SmBSplineCurve*)rPathCurves[ii];

    pPathBefore = NULL;
    pPathAfter  = NULL;
    if ( ii > 0 )
      { pPathBefore = (SmBSplineCurve*)rPathCurves[ii - 1]; }
    if ( ii < rPathCurves.GetSize() - 1 )
      { pPathAfter = (SmBSplineCurve*)rPathCurves[ii + 1]; }

    if( pPathBefore )
    {
      pPathBefore->Evaluate( pPathBefore->GetNaturalInterval().GetMax(), 1, TRUE, sPVBefore );
      pPath->Evaluate( pPath->GetNaturalInterval().GetMin(), 1, FALSE, sPVAfter );

      // do not extend if adjacent path curves are parallel
      if(sPVBefore[1].IsParallelTo( sPVAfter[1], 2.0 ) == 0)
        pPath->CreateExtendedCurve( crContext, dDistance, 1, SM_CT_G1, pPath );
    }
    else if ( bIsPathClosed && !bIsPathPeriodic )
    {
      pPath->CreateExtendedCurve( crContext, dDistance, 1, SM_CT_G1, pPath );
    }

    if( pPathAfter )
    {
      pPath->Evaluate( pPath->GetNaturalInterval().GetMax(), 1, TRUE, sPVBefore );
      pPathAfter->Evaluate( pPathAfter->GetNaturalInterval().GetMin(), 1, FALSE, sPVAfter );

      // do not extend if adjacent path curves are parallel
      if ( sPVBefore[1].IsParallelTo( sPVAfter[1], 2.0 ) == 0 )
        { pPath->CreateExtendedCurve( crContext, dDistance, 2, SM_CT_G1, pPath ); }
    }
    else if( bIsPathClosed && !bIsPathPeriodic )
    {
      pPath->CreateExtendedCurve( crContext, dDistance, 2, SM_CT_G1, pPath );
    }

    rsExtendedPathCurves.Add( (SmCurve*)pPath );
  }

} // end static ExtendPathCurves

/*******************************************************************//**
PURPOSE: Translate and rotate profile curves so they sit at the
         beginning of the path and in the correct orientation

NOTES: Helper function for CreateSweepAlongPlanarPath
***********************************************************************/
static void PositionProfileCurves
 (const SmContext    & crContext,             // in :
  SmTArray<SmCurve*> & rProfileCurves,        // in :
  double               dTol3D,                // NotUsed: in :
  SmCurve            * pPathCurve,            // in :
  SmTArray<SmCurve*> & rsNewProfileCurves)    // out:
{
  SM_REF1(dTol3D) ;
  // Copy profile curves while calculating the bounding box
  SmCurve* pNewProfile = NULL;
  SmExtent3d sBBoxTotal, sBBox;
  ULONG ii;
  for(ii = 0; ii < rProfileCurves.GetSize(); ii++)
  {
    rProfileCurves[ii]->CalculateBoundingBox( rProfileCurves[ii]->GetNaturalInterval(), &sBBox );
    sBBoxTotal.AddPoint3d( sBBox.GetMin() );
    sBBoxTotal.AddPoint3d( sBBox.GetMax() );

    rProfileCurves[ii]->Copy( crContext, pNewProfile );
    rsNewProfileCurves.Add( pNewProfile );
  }

  // Get midpoint of profile curves
  SmVector3d mid = sBBoxTotal.GetMid();

  SmVector3d sXAxis( 1, 0, 0 ), sYAxis( 0, 1, 0 ), sZAxis( 0, 0, 1 );

  // translate profle curves to the origin
  SmAxis2Placement sTransMatrix1;
  sTransMatrix1.SetCanonical( -(mid), sXAxis, sYAxis );

  for(ii = 0; ii < rsNewProfileCurves.GetSize(); ii++)
    { rsNewProfileCurves[ii]->Transform( sTransMatrix1 ); }


  // Calculate normal of profile curves
  SmVector3d sProfileNormal;
  SmCurve::ComputeCrvsNormal( rProfileCurves, sProfileNormal );

  SmAxis2Placement sOrientationMatrix;
  sOrientationMatrix.SetCanonical( SmVector3d( 0, 0, 0 ), sXAxis, sYAxis );

  double dAngle = 0.0;
  sXAxis.AngleBetween( sProfileNormal, dAngle );
  SmVector3d sRotationAxis = sXAxis * sProfileNormal;
  sOrientationMatrix.RotateAboutAxis( -dAngle, sRotationAxis );

  // Orient profle curves to the Y-Z plane.
  for ( ii = 0; ii < rsNewProfileCurves.GetSize(); ii++ )
    { rsNewProfileCurves[ii]->Transform( sOrientationMatrix ); }


  // Calculate the start position and derivative of the path
  SmVector3d startPtDeriv[2];
  pPathCurve->Evaluate( pPathCurve->GetNaturalInterval().GetMin(), 1, TRUE, startPtDeriv );

  // calculate axis at start of path so that xAxis points along path derivative
  sYAxis = sZAxis * startPtDeriv[1];
  if (    smos_Fabs( startPtDeriv[1].z ) > smos_Fabs( startPtDeriv[1].x )
      &&  smos_Fabs( startPtDeriv[1].z ) > smos_Fabs( startPtDeriv[1].y ) )
  {
    sYAxis.Set( 0, 1, 0 );
  }

  startPtDeriv[1].MakeUnitOrthoVectors( &sYAxis, sXAxis, sYAxis, sZAxis );

  SmAxis2Placement sTransMatrix;
  sTransMatrix.SetCanonical( startPtDeriv[0], sXAxis, sYAxis );

  // translate profle curves to the start of the path
  for ( ii = 0; ii < rsNewProfileCurves.GetSize(); ii++ )
    { rsNewProfileCurves[ii]->Transform( sTransMatrix ); }

#ifdef SM_DEBUG_CODE
  SmBoolean bDebugMe = FALSE;
  if(bDebugMe)
  {
    pPathCurve->Draw();
    for(ii = 0; ii < rsNewProfileCurves.GetSize(); ii++)
      { rsNewProfileCurves[ii]->Draw(); }
    sm_GraphicsLoop( TRUE );
  }
#endif // SM_DEBUG_CODE

  return;

} // end static PositionProfileCurves


/*******************************************************************//**
PURPOSE: Sweep profile curves along one path segment. 
         Profiles should already be in position
         Always cap ends and then remove the faces we don't need
         Added bMoveProfileCenterToPath False or True. Historically this was set to TRUE

NOTES: Helper function for CreateSweepAlongPlanarPath
***********************************************************************/
static void SweepProfilesAlongPath
( 
  const SmContext& crContext, 
  SmTArray<SmCurve*>& rProfileCurves,
  SmCurve*  pPathCrv, 
  double    dTol, 
  SmBoolean bCapStart, 
  SmBoolean bCapEnd,
  SmBoolean bMoveProfileCenterToPath, 
  SmBrep*   pBrep 
)
{
  // CreateCurveSweep somehow mangles the path curve
  SmCurve* crvCopy = NULL;
  pPathCrv->Copy( crContext, crvCopy );
  SmBSplineCurve *bsPath = (SmBSplineCurve*)crvCopy;

  SmTArray<SmFace*> startFaces;
  SmTArray<SmFace*> sideFaces;
  SmTArray<SmFace*> endFaces;

  SmBSplineCurve* pScaleReference = NULL;
  SmBSplineCurve* pScaleCurve = NULL;

  SmSweepOptions sOptions;
  sOptions.bCapEndsArg = TRUE;
  sOptions.bTranslationalSweep = FALSE;
  sOptions.bMoveProfileCenterToPath = bMoveProfileCenterToPath;
  sOptions.bOrientProfilePerpendicularToPath = FALSE;

  SmPrimitiveCreation pPC( pBrep->GetInfiniteRegion() );
  pPC.CreateCurveSweep( rProfileCurves, bsPath, pScaleReference, pScaleCurve, dTol,
                        &sOptions, startFaces, sideFaces, endFaces, FALSE );

  if ( bCapStart == FALSE )
  {
    pBrep->m_bEditingEnabled = TRUE;
    pBrep->RemoveFaces( startFaces );  // increments unlocked mark value
  }

  if ( bCapEnd == FALSE )
  {
    pBrep->m_bEditingEnabled = TRUE;
    pBrep->RemoveFaces( endFaces );  // increments unlocked mark value
  }

  // should we stitch here???
  //pBrep->StitchAndOrient();

} // end static SweepProfilesAlongPath

/*******************************************************************//**
  PURPOSE: Create series of breps by sweeping a closed planar profile
  along a planar path. Both the profile and the path can be a set
  of curves. Both the profile and the path can be closed or open.

  NOTES: Added bMoveProfileCenterToPath. Historically this was set to TRUE
***********************************************************************/
SmStatus SmPrimitiveCreation::CreateSweepAlongPlanarPath
 (const SmContext    & crContext,                  // in : 
  SmTArray<SmCurve*> & rProfileCurves,             // in : 
  SmTArray<SmCurve*> & rPathCurves,                // in : 
  double               dTol,                       // in : 
  SmBoolean            bCapEndsArg,                // NotUsed: in : 
  SmBoolean            bMoveProfileCenterToPath,   // in : 
  SmBrep            *& pSweepBrep)                 // out: 
{
  SM_REF1(bCapEndsArg) ;
  return CreateSweepAlongPlanarPath(crContext,
                                    rProfileCurves,
                                    rPathCurves,
                                    dTol,
                                    TRUE,
                                    TRUE,
                                    bMoveProfileCenterToPath,
                                    pSweepBrep);
}

/*******************************************************************//**
  PURPOSE: Create a planar-path sweep with independent control of the
  start and end caps. Closed paths have no endpoint caps.
***********************************************************************/
SmStatus SmPrimitiveCreation::CreateSweepAlongPlanarPath
 (const SmContext    & crContext,                  // in :
  SmTArray<SmCurve*> & rProfileCurves,             // in :
  SmTArray<SmCurve*> & rPathCurves,                // in :
  double               dTol,                       // in :
  SmBoolean            bCapStartArg,               // in :
  SmBoolean            bCapEndArg,                 // in :
  SmBoolean            bMoveProfileCenterToPath,   // in :
  SmBrep            *& pSweepBrep)                 // out:
{
  SmTArray<SmBrep*> sNewBreps;
  ULONG ii;

  // Determine whether path curves form a closed and periodic loop.
  SmVector3d sStartPV[2], sEndPV[2];
  rPathCurves[0]->Evaluate( rPathCurves[0]->GetNaturalInterval().GetMin(), 1, 1, sStartPV );
  rPathCurves[rPathCurves.GetSize() - 1]->Evaluate( rPathCurves[rPathCurves.GetSize() - 1]->GetNaturalInterval().GetMax(), 1, 1, sEndPV );

  SmBoolean bIsPathClosed = FALSE;
  if ( sStartPV[0].DistanceBetween( sEndPV[0] ) < dTol )
    { bIsPathClosed = TRUE; }

  SmBoolean bIsPathPeriodic = FALSE;
  if ( bIsPathClosed && sStartPV[1].IsParallelTo( sEndPV[1], 2.0 ))
    { bIsPathPeriodic = TRUE; }

  // Determine max size of profile (we may not need this)
  SmExtent3d sBBoxTotal, sBBox;
  for( ii = 0; ii < rProfileCurves.GetSize(); ii++ )
  {
    rProfileCurves[ii]->CalculateBoundingBox( rProfileCurves[ii]->GetNaturalInterval(), &sBBox );
    sBBoxTotal.AddPoint3d( sBBox.GetMin() );
    sBBoxTotal.AddPoint3d( sBBox.GetMax() );
  }

  double dMaxDimension = sBBoxTotal.GetMaxDimension();
//double dDistToExtend = smos_Sqrt( (dMaxDimension * dMaxDimension) + (dMaxDimension * dMaxDimension) );
  double dDistToExtend = dMaxDimension * smos_Sqrt( 2 );
  dDistToExtend = dDistToExtend * 2;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe=FALSE;
  if ( bDebugMe ) {
      smgfx_Erase();
      for(ULONG di = 0; di < rPathCurves.GetSize(); di++)
        { smgfx_ChangeColor(di != 0); rPathCurves[di]->Draw(); sm_GraphicsLoop(); }
      sm_GraphicsLoop();
  }
#endif // SM_DEBUG_CODE

  // Extend all the path curves at start and end to make sure sweeps go past corner
  // We cannot go too far for fear of causing second intersection
  SmTArray <SmCurve*> sExtendedPathCurves;
  ExtendPathCurves( crContext,
                    rPathCurves,
                    bIsPathClosed,
                    bIsPathPeriodic,
                    dDistToExtend,
                    sExtendedPathCurves );

#ifdef SM_DEBUG_CODE
  if ( bDebugMe ) {
      smgfx_Erase();
      for(ULONG di = 0; di < sExtendedPathCurves.GetSize(); di++)
        { smgfx_ChangeColor(di != 0); sExtendedPathCurves[di]->Draw(); sm_GraphicsLoop(); }
      sm_GraphicsLoop();
  }
#endif // SM_DEBUG_CODE

  // get the normal to the plane of the path
  SmVector3d sPathNormal;
  SmCurve::ComputeCrvsNormal( rPathCurves, sPathNormal );

  // Prepare for loop where we position profile curves, sweep, and trim
  SmTArray<SmCurve*> sNewProfileCurves;
  SmBrep* pBrepBefore = NULL;
  SmBrep* pBrepAfter = NULL;
  SmBrep* pBrepFirst = NULL;
  SmCurve* pPath = NULL;
  SmCurve* pExtendedPath = NULL;
  SmBSplineCurve* pCutLine = NULL;
  SmBSplineCurve* pCuttingCrv = NULL;
  SmBSplineCurve* pTempCuttingCrv = NULL;

  SmVector3d sPVBefore[2], sPVAfter[2];
  SmVector3d sPVFirst[2];
  SmVector3d sRefPoint1,  sRefPoint2;
  SmVector3d sAverageDir, sCuttingPt;
  SmExtent1d sPathRange;

  SmBoolean bCapStart, bCapEnd;
  double dAngleBetween;

  // Determine if adjacent path curves are both linear.
  // If so, then current algortithm works well.
  // Otherwise lets try and take brep intersection.
  for ( ii = 0; ii < rPathCurves.GetSize(); ii++ )
  {

    pPath = rPathCurves[ii];
    pExtendedPath = sExtendedPathCurves[ii];

    // find start point and derivative of new path segment
    sPathRange = pPath->GetNaturalInterval();
    pPath->Evaluate( sPathRange.GetMin(), 1, FALSE, sPVAfter );
    sPVAfter[1].Unitize();

    // position profile curves at the start of the path segment
    sNewProfileCurves.RemoveAll();
    PositionProfileCurves( crContext, rProfileCurves, dTol, pExtendedPath, sNewProfileCurves );

    // sweep the profiles along this path segment creating new brep
    pBrepAfter = new (crContext)SmBrep();

    bCapStart = bCapEnd = FALSE;
    if ( ii == 0 && bIsPathClosed == FALSE && bCapStartArg )
      { bCapStart = TRUE; }

    if ( ii == rPathCurves.GetSize() - 1 && bIsPathClosed == FALSE && bCapEndArg )
      { bCapEnd = TRUE; }

#ifdef SM_DEBUG_CODE
    if ( bDebugMe ) {
        smgfx_Erase();
        for(ULONG di = 0; di < sNewProfileCurves.GetSize(); di++)
        {
          smgfx_ChangeColor(di != 0);  sNewProfileCurves[di]->Draw(); sm_GraphicsLoop();
        }
        smgfx_SetLook( 3, 3, 0, 0, 1 );  pExtendedPath->Draw(); sm_GraphicsLoop();
        sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

    // Do the sweep.
    SweepProfilesAlongPath( crContext,
                            sNewProfileCurves,
                            pExtendedPath,
                            dTol,
                            bCapStart,
                            bCapEnd,
                            bMoveProfileCenterToPath,
                            pBrepAfter );

#ifdef SM_DEBUG_CODE
    if ( bDebugMe ) {
        smgfx_Erase();
        smgfx_ChangeColor(ii != 0);  pBrepAfter->Draw(); sm_GraphicsLoop();
        sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

    sNewBreps.Add( pBrepAfter );

    // If this is the first segment then just keep track of derivatives, etc.
    // Otherwise, cut the brep on either side at the '45'.
    if ( ii == 0 )
    {
      pBrepFirst  = pBrepAfter;
      sPVFirst[0] = sPVAfter[0];
      sPVFirst[1] = sPVAfter[1];
    }
    else
    {

      // If derivatives are not parallel (paths are tangent continuous), then we need to trim both breps.
      if ( ! sPVBefore[1].IsParallelTo( sPVAfter[1], 2.0 ) )
      {
        // Average the derivatives to determine cutting plane normal.
        sAverageDir = (-sPVBefore[1]) + sPVAfter[1];
        sAverageDir.Unitize();

        // The angleBetween the derivatives helps us determine if this is an outer or inner corner.
        // If this is an inner corner than flip the average direction so we can extend correctly.
        sPathNormal.CCWAngleBetween( (-sPVBefore[1]), sPVAfter[1], dAngleBetween );
        dAngleBetween = SM_RAD2DEG( dAngleBetween );
        if ( dAngleBetween > 0 )
          { sAverageDir = -sAverageDir; }

        // Create temp cutting curve. This will work if adjacent paths are lines.

        // Find approx end point of cutting curve by using average deraivative
        sCuttingPt = sPVAfter[0] + ( sAverageDir * dDistToExtend );

        // Creating a line from the corner where the paths meet to the approx end point
        SmBSplineCurve::CreateLineSegment( crContext, 3, sPVAfter[0], sCuttingPt, pCutLine );

        // Now extend the line backwards
        pCutLine->CreateExtendedCurve( crContext, dDistToExtend, 1, SM_CT_G1, pTempCuttingCrv );

#ifdef SM_DEBUG_CODE
        if(bDebugMe)
        {
          smgfx_SetLook( 4, 6, 1, 0, 0 );  pBrepAfter->Draw(); sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE

        // Set pCuttingCurve.
        pCuttingCrv = NULL;

        // If adjacent paths are not lines then we can temp pCuttingCrv to get real cutting curve.
        // We do this by intersecting the temp cutting curve with every intersection curve.
        // After finding the two best intersections, we can create a real curve.
        if ( rPathCurves[ii]->IsLinear( dTol ) && rPathCurves[ii-1]->IsLinear( dTol ) )
        {
          pCuttingCrv = pTempCuttingCrv;
        }
        else
        {
          SmTArray <SmCurve*> sIntersectionCrvs;
          pBrepBefore->BrepIntersect( crContext, pBrepAfter, dTol, 2.0, sIntersectionCrvs, NULL );

#ifdef SM_DEBUG_CODE
          if(bDebugMe)
          {
            smgfx_Erase();
            pBrepBefore->Draw(); sm_GraphicsLoop();
            pBrepAfter ->Draw(); sm_GraphicsLoop();

            smgfx_SetLook( 2,3, 1,0,1 );
            for(ULONG di = 0; di < sIntersectionCrvs.GetSize(); di++)
            {
              sIntersectionCrvs[di]->Draw(); sm_GraphicsLoop();
              sm_GraphicsLoop();
            }
          }
#endif // SM_DEBUG_CODE

          ULONG jj, lNumIntCurves = sIntersectionCrvs.GetSize();
          if ( lNumIntCurves > 1 )  // This won't work with only one intersection.
          {
            SmPoint3d sBestPt;
            SmPoint3d sBestPt2;
            double dBestDist  = SM_BIG_DOUBLE;
            double dBestDist2 = SM_BIG_DOUBLE;
            for(jj = 0; jj < lNumIntCurves; jj++)
            {
              SmSolutionArray sSolutions;
              pTempCuttingCrv->GlobalCurveSolve( pTempCuttingCrv->GetNaturalInterval(), *sIntersectionCrvs[jj], sIntersectionCrvs[jj]->GetNaturalInterval(), SM_SO_MINIMIZE, dTol, NULL, NULL, SM_SR_SINGLE, sSolutions );

              double dDist;
              for(ULONG kk = 0; kk < sSolutions.GetSize(); kk++)
              {
                SmSolution & rSol = sSolutions[kk];

                SmPoint3d sPtOnLine;
                rSol.GetPoint( pTempCuttingCrv, TRUE, sPtOnLine );

                SmPoint3d sPtOnIntersectionCrv;
                rSol.GetPoint( sIntersectionCrvs[jj], TRUE, sPtOnIntersectionCrv );

                dDist = sPtOnIntersectionCrv.DistanceBetween( sPtOnLine );

                // keep track of the best two solutions
                if(dDist < dBestDist)
                {
                  // Promote sBestPt2.
                  if ( sBestPt.IsInitialized() )
                    { sBestPt2 = sBestPt; }
                  dBestDist2 = dBestDist;

                  sBestPt = sPtOnIntersectionCrv;
                  dBestDist = dDist;
                }
                else if(dDist < dBestDist2)
                {
                  sBestPt2 = sPtOnIntersectionCrv;
                  dBestDist2 = dDist;
                }
              }
            } // for each intersection crv

#ifdef SM_DEBUG_CODE
            if(bDebugMe)
            {
              smgfx_Erase();
              smgfx_SetLook( 3,3, 0,0,1 );
              pBrepBefore->Draw(); sm_GraphicsLoop();
              pBrepAfter ->Draw(); sm_GraphicsLoop();

              smgfx_SetLook( 2,3, 0,1,0 );
              pTempCuttingCrv->Draw(); sm_GraphicsLoop();

              smgfx_SetLook( 3,8, 1,0,1 );
              sBestPt .Draw(); sm_GraphicsLoop();
              sBestPt2.Draw(); sm_GraphicsLoop();
              sm_GraphicsLoop();
            }
#endif // SM_DEBUG_CODE

            if ( sBestPt.IsInitialized() && sBestPt2.IsInitialized() )
              {
                if ( sBestPt.DistanceBetween( sBestPt2 ) > dTol * 10.0 )
                  {
                    SmBSplineCurve* pCrv = NULL;
                    SmBSplineCurve::CreateLineSegment( crContext, 3, sBestPt, sBestPt2, pCrv );
                    pCrv->CreateExtendedCurve( crContext, dDistToExtend, 0, SM_CT_G1, pCrv );
                    pCrv->CreateExtendedCurve( crContext, dDistToExtend, 1, SM_CT_G1, pCuttingCrv );
                  }
              }

          } // end if there were Brep intersections

        } // end create cutting line for non linear paths

        if ( pCuttingCrv == NULL )
          { pCuttingCrv = pTempCuttingCrv; } // Just as a backup: this should still work.


        if ( pCuttingCrv != NULL )
        {
          // determine reference points on either side of the cutting line
          sRefPoint1 = sPVBefore[0] + (-sPVBefore[1] * 0.1);
          sRefPoint2 = sPVAfter [0] + ( sPVAfter [1] * 0.1);

#ifdef SM_DEBUG_CODE
          if(bDebugMe)
          {
            smgfx_Erase();
            smgfx_SetLook( 1,3, 0,0,1 ); pBrepBefore->Draw(); sm_GraphicsLoop();
            smgfx_SetLook( 1,3, 0,1,0 ); pBrepAfter ->Draw(); sm_GraphicsLoop();

            smgfx_SetLook( 2,3, 1,0,1 ); pCuttingCrv->Draw(); sm_GraphicsLoop();

            smgfx_SetLook( 3,5, 1,0,0 ); sPVAfter[0].Draw(); sm_GraphicsLoop();

            smgfx_SetLook( 3,5, 0,0,1 ); sRefPoint1.Draw(); sm_GraphicsLoop();
            smgfx_SetLook( 3,5, 0,1,0 ); sRefPoint2.Draw(); sm_GraphicsLoop();
            sm_GraphicsLoop();
          }
#endif // SM_DEBUG_CODE

          pBrepBefore->ProjectAndTrim( *pCuttingCrv, // can increment an unused Mark value
                                       sPathNormal,
                                       sRefPoint2,
                                       SM_TT_DELETE_POINT,
                                       &dTol, NULL, NULL );

          pBrepAfter->ProjectAndTrim( *pCuttingCrv,  // can increment an unused Mark value
                                      sPathNormal,
                                      sRefPoint1,
                                      SM_TT_DELETE_POINT,
                                      &dTol, NULL, NULL );

        } // end if pCuttingCurve was created

      } // end if tangents are not parallel

    } // end else ( ii>0 )

    // Get ready for next path segment
    pBrepBefore = pBrepAfter;

    // find end derivative of path before
    pPath->Evaluate( sPathRange.GetMax(), 1, TRUE, sPVBefore );
    sPVBefore[1].Unitize();

    // is path is a closed loop, trim end of last and beginning of first 
    if ( bIsPathClosed && ii == rPathCurves.GetSize() - 1 )
    {
      pBrepAfter  = pBrepFirst;
      sPVAfter[0] = sPVFirst[0];
      sPVAfter[1] = sPVFirst[1];

      // if derivatives are parallel (paths are tangent continuous), then we don't need to trim
      if ( ! sPVBefore[1].IsParallelTo( sPVAfter[1], 2.0 ) )
      {
        // Average the derivatives to determine cutting plane normal.
        sPVBefore[1].Unitize();
        sPVAfter [1].Unitize();
        sAverageDir = (-sPVBefore[1]) + sPVAfter[1];

        sPathNormal.CCWAngleBetween( (-sPVBefore[1]), sPVAfter[1], dAngleBetween );
        dAngleBetween = SM_RAD2DEG( dAngleBetween );
        if(dAngleBetween > 0)
          { sAverageDir = -sAverageDir; }

        // The distance to extend the cutting line can be tricky.
        // For most cases, the longer the better (50) to ensure it goes all the way through,
        // but there are cases where a long cutting curve will intersect the breps in two places.
        // How can I determine the correct distance???
        sCuttingPt = sPVAfter[0] + (sAverageDir * 50);
        SmBSplineCurve::CreateLineSegment( crContext, 3, sPVAfter[0], sCuttingPt, pCutLine );
        pCutLine->CreateExtendedCurve( crContext, 50, 1, SM_CT_G1, pCuttingCrv );

        // determine reference points on either side of the cutting line
        if(pBrepBefore == pBrepAfter)
        {
          sRefPoint1 = sPVBefore[0] + ( sPVBefore[1] * 10);
          sRefPoint2 = sPVAfter [0] + (-sPVAfter [1] * 10);
        }
        else
        {
          sRefPoint1 = sPVBefore[0] + (-sPVBefore[1] * 30);
          sRefPoint2 = sPVAfter [0] + ( sPVAfter [1] * 30);
        }

#ifdef SM_DEBUG_CODE
        if(bDebugMe)
        {
          smgfx_Erase();
          smgfx_SetLook( 1,2, 0,0,0 ); pBrepBefore->Draw(TRUE); sm_GraphicsLoop();
          //pBrepAfter->Draw(TRUE);
          smgfx_SetLook( 2,4, 1,0,0 ); pCuttingCrv->Draw(); sm_GraphicsLoop();

          smgfx_SetLook( 3,15, 0,1,0 ); sPVAfter[0].Draw(); sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE
        if ( pBrepBefore == pBrepAfter )
        {

          ULONG nEdges, nLaminaEdges, jj, kk;
          double dMaxVertexGap = 0.0, dMaxEdgeGap = 0.0;
          pBrepBefore->StitchFaces( 0.01, nEdges, nLaminaEdges, dMaxVertexGap, dMaxEdgeGap );

          pBrepBefore->ProjectAndTrim( *pCuttingCrv,   // can increment an unused Mark value
                                       sPathNormal,
                                       sRefPoint2,
                                       SM_TT_SPLIT,
                                       &dTol, NULL, NULL );

          SmTArray<SmFace*> sRemoveFaces;
          SmTArray<SmFace*> sFaces;
          SmTArray<SmEdge*> sEdges;

          pBrepBefore->GetEdges( sEdges );
          for( jj = 0; jj < sEdges.GetSize(); jj++)
          {
            if ( sEdges[jj]->IsLamina() )
            {
              sEdges[jj]->GetFaces( sFaces );
              for ( kk = 0; kk < sFaces.GetSize(); kk++ )
                sRemoveFaces.Add( sFaces[kk] );
            }
          }

          pBrepBefore->RemoveFaces( sRemoveFaces );  // increments unlocked mark value

          pBrepBefore->RemoveTopologicalEdgesAndVertices();
        }
        else
        {
          pBrepBefore->ProjectAndTrim( *pCuttingCrv,         // can increment an unused Mark value
                                       sPathNormal,
                                       sRefPoint2,
                                       SM_TT_DELETE_POINT,
                                       &dTol, NULL, NULL );

          pBrepAfter->ProjectAndTrim( *pCuttingCrv,          // can increment an unused Mark value
                                      sPathNormal,
                                      sRefPoint1,
                                      SM_TT_DELETE_POINT,
                                      &dTol, NULL, NULL );
        }
      } // end if adjacent segments are not parallel
    } // end if path is closed, trim end of last and beginning of first 
  } // end for each rPathCurve

  // merge all results into one brep
  SmTArray<SmBrep*> sOrderedBreps;
  ULONG lCount = sNewBreps.GetSize();
  SmTArray<long> sBoolTree;

  sBoolTree.Add( 0 );
  sOrderedBreps.Add( sNewBreps.GetLast() );
  sNewBreps.RemoveLast();
  for( ii = 1; ii < lCount; ii++)
  {
    sOrderedBreps.Add( sNewBreps.GetLast() );
    sNewBreps.RemoveLast();
    sBoolTree.Add( ii );
    sBoolTree.Add( 0 - 1 ); // Convert to different form of Boolean numbering
  }

  SER( SmMerge::BooleanTrees( sOrderedBreps, sBoolTree ) );
  if ( sOrderedBreps.GetSize() != 1 )
    { SER( SM_ERR ); } // Something went wrong

  pSweepBrep = sOrderedBreps[0];

  ULONG nEdges, nLaminaEdges;
  double dMaxVertexGap = 0.0, dMaxEdgeGap = 0.0;
  SER_DELETE( pSweepBrep->StitchFaces( dTol, nEdges, nLaminaEdges, dMaxVertexGap, dMaxEdgeGap ),
              pSweepBrep );

  pSweepBrep->RemoveTopologicalEdgesAndVertices();

  // A second attempt can remove additional edges
  pSweepBrep->RemoveTopologicalEdgesAndVertices();

  // A closed path, or an open path with both endpoint caps, expresses solid
  // intent when the swept profile closes into a manifold boundary.  Boolean
  // assembly can preserve the correct boundary while leaving every region
  // marked void.  Apply the standard nested-solid parity classification so
  // bounded regions alternate between material and void from the infinite
  // region inward.  Open sheets and partially capped shells remain void.
  const SmBoolean bSolidIntent = bIsPathClosed || ( bCapStartArg && bCapEndArg );
  const SmBoolean bIsManifoldSolid = pSweepBrep->IsManifoldSolid();
  if( bSolidIntent && bIsManifoldSolid )
    { SER_DELETE( pSweepBrep->SetRegionIsVoidFlagsForNestedSolids(), pSweepBrep ); }

  // Defensively reject any remaining manifold boundary whose regions are all
  // void rather than publishing a zero-volume result as a solid success.
  SmTArray<SmRegion*> sRegions;
  pSweepBrep->GetRegions(sRegions);
  SmBoolean bHasMaterialRegion = FALSE;
  for(ULONG jj=0; jj<sRegions.GetSize(); jj++)
    if(!sRegions[jj]->IsVoid())
      {
        bHasMaterialRegion = TRUE;
        break;
      }
  if(bIsManifoldSolid && !bHasMaterialRegion)
    {
      delete pSweepBrep;
      pSweepBrep = NULL;
      SER(SM_ERR);
    }

  return SM_SUCCESS;

} // end CreateSweepAlongPlanarPath

static SmStatus sm_CreateSkinPrimitiveInternal
 (SmPrimitiveCreation   & rPrimitiveCreation,
  SmRegion              * pRegion,
  const SmTArray<ULONG>  & rCurvesInEachProfile,
  SmTArray<SmCurve*>     & r3DCurves,
  double                   bAppoxTol3d,
  ULONG                    lDegree,
  SmBoolean                bCapEnds,
  SmTArray<SmFace*>      & rStartFaces,
  SmTArray<SmFace*>      & rSideFaces,
  SmTArray<SmFace*>      & rEndFaces,
  SmTArray<SmBoolean>    * pOptInputOwnershipTransferred);

/*******************************************************************//**
PURPOSE: Create a sheet or a solid model within the contained
            Brep by lofting a set of profiles.  

NOTES:
  A set of 3D Curves is organized into profiles.  Each profile
  may contain any number of curves.  The curves from one profile to
  the next are correlated.  Loft surfaces are built and turned into
  faces for every set of correlated curves starting at the first
  profile and moving through the 2nd and so on to the last profile.

  When the profiles consist of 3d coplanar curves that can be organized
  into loops on a planar face, then cap surfaces can be requested that
  will turn the contained brep into a closed solid model.
  
  When the profiles consist of curves that can not be organized into
  closed loops, no end cap faces can be built and the contained
  brep becomes a sheet model.
  
  When bAppoxTol3d = 0.0, the loft surfaces interpoate the profile curves.
  When bAppoxTol3d > 0.0, the loft surfaces approximate the profiles curves.
    Often times approximate loft surfaces will be smoother than exact loft surfaces.

  lDegree specifes the degree of the loft surface in the loft direction.
  Loft surfaces will be piecewise planar, parabolic, or cubic as,
  1 = planar, 2 = parabolic, 3 = cubic.

  The contained Brep is modified with the addition of all side and cap faces.
  The list of lofted sideFaces, and start and end cap faces are returned.
  
***********************************************************************/
SmStatus SmPrimitiveCreation::CreateSkinPrimitive
 (SmTArray<ULONG>    & rCurvesInEachProfile, // in : Number of curves in each profile
  SmTArray<SmCurve*> & r3DCurves,            // in : Array of all profile segments
  double               bAppoxTol3d,          // in : 0.0 = loft surfaces interpolate loft curves,
                                             //      else loft surfaces approximate loft surfaces.
  ULONG                lDegree,              // in : degree of loft surfaces in loft direction, 1=linear,2=parabolic,3=cubic
  SmBoolean            bCapEnds,             // in : TRUE = cap first and last profiles, FALSE = no cap faces
  SmTArray<SmFace*>  & rStartFaces,          // out: list of start cap faces
  SmTArray<SmFace*>  & rSideFaces,           // out: list of side faces across all loft spans
  SmTArray<SmFace*>  & rEndFaces)            // out: list of end cap faces
{
  SER(sm_CreateSkinPrimitiveInternal(*this,
                                     m_pRegion,
                                     rCurvesInEachProfile,
                                     r3DCurves,
                                     bAppoxTol3d,
                                     lDegree,
                                     bCapEnds,
                                     rStartFaces,
                                     rSideFaces,
                                     rEndFaces,
                                     NULL));
  return SM_SUCCESS;
}

/*******************************************************************//**
PURPOSE: Create a skin and report which input curve pointers were consumed.

NOTES:
  This preserves CreateSkinPrimitive's legacy geometry and ownership behavior
  while allowing adapters with private working copies to reclaim any curves
  left unconsumed on an error path.
***********************************************************************/
SmStatus SmPrimitiveCreation::CreateSkinPrimitive
 (const SmTArray<ULONG>& rCurvesInEachProfile,
  SmTArray<SmCurve*>  & r3DCurves,
  double                bAppoxTol3d,
  ULONG                 lDegree,
  SmBoolean             bCapEnds,
  SmTArray<SmFace*>   & rStartFaces,
  SmTArray<SmFace*>   & rSideFaces,
  SmTArray<SmFace*>   & rEndFaces,
  SmTArray<SmBoolean> & rInputOwnershipTransferred)
{
  SER(sm_CreateSkinPrimitiveInternal(*this,
                                     m_pRegion,
                                     rCurvesInEachProfile,
                                     r3DCurves,
                                     bAppoxTol3d,
                                     lDegree,
                                     bCapEnds,
                                     rStartFaces,
                                     rSideFaces,
                                     rEndFaces,
                                     &rInputOwnershipTransferred));
  return SM_SUCCESS;
}

/*******************************************************************//**
PURPOSE: Shared skin implementation with optional input-ownership reporting.

NOTES:
  The legacy entry point supplies no report and retains its historical
  behavior. The reporting entry point stops if a failed planar-profile
  conversion has already transferred any curves, allowing its adapter to
  reclaim only the private copies which remain unowned.
***********************************************************************/
static SmStatus sm_CreateSkinPrimitiveInternal
 (SmPrimitiveCreation   & rPrimitiveCreation,
  SmRegion              * pRegion,
  const SmTArray<ULONG>  & rCurvesInEachProfile,
  SmTArray<SmCurve*>     & r3DCurves,
  double                   bAppoxTol3d,
  ULONG                    lDegree,
  SmBoolean                bCapEnds,
  SmTArray<SmFace*>      & rStartFaces,
  SmTArray<SmFace*>      & rSideFaces,
  SmTArray<SmFace*>      & rEndFaces,
  SmTArray<SmBoolean>    * pOptInputOwnershipTransferred)
{
  // empty entity maps - m_vMapToSame, m_vMapToHigher, m_vMapFromSame, and m_vMapFromLower
  rPrimitiveCreation.InitEntityMaps();

  // init outputs
  rStartFaces.ReSet();
  rEndFaces.ReSet();
  rSideFaces.ReSet();
  if(pOptInputOwnershipTransferred)
    {
      pOptInputOwnershipTransferred->SetSize(r3DCurves.GetSize());
      pOptInputOwnershipTransferred->SetAll(FALSE);
    }

  // check input, at least two profiles
  if (rCurvesInEachProfile.GetSize() < 2) SER(SM_ERR);

  // local region, brep, tol and context values
  SmBrep          *pBrep    = pRegion->GetBrep();
  const SmContext *pContext = pBrep->GetContext();
  double           dTol     = pBrep->GetTolerance();

  // tell brep it's going to be modified
  pBrep->m_bEditingEnabled = TRUE;
  
  // and that we'll be doing a Boolean
  SmTemporaryChangeValue< SmBoolean > sDoingBool( SM_CONST_CAST(SmContext*, pContext)->GetDoingBooleanRef(), TRUE );

  // make scratch brep  
  SmBrep *pTmpBrep = new (*pBrep->GetContext()) SmBrep();
  SmObjDelete sCleanBrep(pTmpBrep);
  // gwc: removed next line - sets pTmpBrep->Tol = *pBrep->GetContext()::ZoneTol3d
  //    pTmpBrep->SetTolerance(dTol);
  pTmpBrep->m_bEditingEnabled = TRUE;

  // construction locals
  ULONG                     lCount = 0;
  SmTArray<SmFace*>         sFaces;           //      
  SmTArray<SmCurve*>        sProfileCurves;   //      
  SmTArray<SmBSplineCurve*> sProfileNurbs;    //      
  SmTArray<SmFace*>         sProfileFaces;    //      
  SmTArray<SmBSplineCurve*> sNurbs;           //      

  // profiles are boundaries of planar faces - they must be closed
  SmBoolean bOpenProfiles = FALSE;

  // for every input profile - 
  //     1. build a planar face bounded by the profiles (if profiles are closed and planar),
  //  or 2. build a single composite curve to be lofted (if 1st profile is non-planar or open)
  for (ULONG ii=0; ii<rCurvesInEachProfile.GetSize(); ii++) 
    {
      sProfileCurves.ReSet();
      sProfileNurbs.ReSet();
      const ULONG lProfileStart = lCount;

      // for every curve in this profile, load Profile arrays
      for (ULONG j=0; j<rCurvesInEachProfile[ii]; j++) 
        {
          sProfileCurves.Add(r3DCurves[lCount]);
          sProfileNurbs.Add((SmBSplineCurve*)r3DCurves[lCount++]);
        }

      // On first iteration or
      // once we've tested the 1st input profile
      if(   ii == 0                  // assume input profiles are loops on planar faces
         || bOpenProfiles == FALSE)  // know   input profiles are loops on planar faces
        {
          // see if next profile is a set of co-planar loops.
          // this call fails for non-planar geometry and open loops (plus degenerate curves, and spline points)
          SmTArray<SmObject*> sOwnersBefore;
          if(pOptInputOwnershipTransferred)
            {
              for(ULONG jj=lProfileStart; jj<lCount; jj++)
                { sOwnersBefore.Add(r3DCurves[jj]->GetOwner()); }
            }

          SmStatus ePlanarStat = pTmpBrep->CreatePlanarFacesWith3DCurves
                                           (pTmpBrep->GetInfiniteRegion(),
                                            sProfileCurves,
                                            dTol,
                                            sProfileFaces);

          // A lower-level error can occur after one or more curves have
          // already transferred to lower-level topology. Record those
          // transfers while their owner pointers are still live.
          SmBoolean bPartialTransfer = FALSE;
          if(pOptInputOwnershipTransferred)
            {
              for(ULONG jj=lProfileStart; jj<lCount; jj++)
                {
                  if(r3DCurves[jj]->GetOwner() != sOwnersBefore[jj-lProfileStart])
                    {
                      (*pOptInputOwnershipTransferred)[jj] = TRUE;
                      bPartialTransfer = TRUE;
                    }
                }
            }

          if(SM_SUCCESS != ePlanarStat)
            {
              // The ownership-reporting path must not reinterpret a partially
              // transferred closed profile as an open profile. The lower-level
              // operation owns those pointers, so the adapter must not reclaim them.
              if(pOptInputOwnershipTransferred && bPartialTransfer)
                SER(ePlanarStat);

              // if the first profile is not a set of co-planar loops
              if(ii == 0) 
                {
                  // treat input data like a composite curve
                  bOpenProfiles = TRUE ;
                }
              else // some subsequent profile failed to be a set of coplanar loops
                {
                  // input profiles don't all match.
                  SER(SM_ERR) ;
                }
            } // end CreatePlanarFacesWith3DCurves failure branch
          else
            {
              // CreatePlanarFacesWith3DCurves consumes every curve only when
              // it succeeds. Record the transfer before any later error.
              if(pOptInputOwnershipTransferred)
                {
                  for(ULONG jj=lProfileStart; jj<lCount; jj++)
                    { (*pOptInputOwnershipTransferred)[jj] = TRUE; }
                }
            }

          // error - more than one face per profile
          if (sProfileFaces.GetSize() > 1) SER(SM_ERR);

          // add to face list
          sFaces.Append(sProfileFaces);

        } // end input profiles are loops on planes case

      // when input data has been found to be a sequence of loft curves
      if(bOpenProfiles == TRUE)
        {
          // build a composite curve from these profile curves
          double dBrepTol   = pBrep->GetTolerance();
          SmTArray<SmCompositeCurve*> sComposites;
          SmStatus eCompositeStat = SmCompositeCurve::BuildCompositesFromCurves
                  (*pContext,       // in : context for new object construction
                    sProfileNurbs,  // in : unordered curves to connect into composites                                  
                                    //      Included curves move to the output composites.
                    TRUE,           // in : (not yet used) bMakeCurvesHomogeneous : TRUE = Approx curves with deg 3 NUBs
                    0.0,            // in : (not yet used) dThisApproxTol3d = tol used if an approximation is required.
                    dBrepTol,       // in : if      gap < dSamePointTolerance, connect 
                    0.0,            // in : (not yet used) else if gap < dOptDistanceToAverage,    snap to avg and connect - not implemented
                    0.0,            // in : (not yet used) else if gap < dOptDistanceToExtendTrim, trim and connect        - not implemented
                    0.0,            // in : else if gap < dOptDistanceToCreateLine, insert line seg and connect
                    0.0,            // in : (not yet used) else if gap < dOptDistanceToCreateBlend, insert blend and connect
                    sComposites);   // out: Composite Curve(s) owning their curve segments.
          SmObjsDelete<SmCompositeCurve*> sClean(&sComposites) ;
          SER(eCompositeStat);

          if(pOptInputOwnershipTransferred)
            {
              // A tiny curve may be omitted rather than transferred. Inspect
              // the composites so ownership is reported per included pointer.
              for(ULONG jj=0; jj<sComposites.GetSize(); jj++)
                {
                  ULONG lDimension = 0;
                  SmBoolean bClosed = FALSE;
                  SmTArray<SmCurve*> sCanonicalCurves;
                  SER(sComposites[jj]->GetCanonical(lDimension, sCanonicalCurves, bClosed));
                  for(ULONG kk=0; kk<sCanonicalCurves.GetSize(); kk++)
                    {
                      for(ULONG ll=lProfileStart; ll<lCount; ll++)
                        {
                          if(sCanonicalCurves[kk] == r3DCurves[ll])
                            {
                              (*pOptInputOwnershipTransferred)[ll] = TRUE;
                              break;
                            }
                        }
                    }
                }
            }

          if (sComposites.GetSize() != 1) SER(SM_ERR);

          // build a SmBSplineCurve for each composite curve
          SmCompositeCurve *pComp = sComposites[0];
          SmBSplineCurve *pCompNurb = NULL ;
          SER(pComp->MakeCompositeNurb(*pContext,pCompNurb));

          // add the single SmBSplineCurve to the sNurbs list
          sNurbs.Add(pCompNurb);
        }

    } // end iter every input profile - building CreateSkinFromFaces inputs

  // when data was found to be a sequence of loft curves
  if(bOpenProfiles == TRUE) 
    {
      // don't make cap faces
      bCapEnds = FALSE;

      // removing memory leak from SmComposites branch turned this into a double delete
      // SmObjsDelete<SmCurve*> sCleanCurves(&r3DCurves);  // GWC ??? memory leak

      // clear out the pointers of the r3DCurves array - they're stale
      r3DCurves.SetAll(NULL) ;
    }

  // loft the loops on a sequence of faces or loft a sequence of loft curves - makes rSideFaces
  SER(rPrimitiveCreation.CreateSkinFromFaces(sFaces, // in : list of faces with maching loops to loft
                          sNurbs,                   // in : when sFaces is empty, a sequence of target loft curves
                          bAppoxTol3d,              // in : 0.0 = loft surface interpolates the loftc curves, 
                                                    //      else loft surface appoxs the loft curves
                          lDegree,                  // in : degree of loft direction, 1 = linear, 2=parabolic, 3=cubic
                          rSideFaces));             // out: list of side faces


  // When asked - create start and stop cap faces  
  if (bCapEnds) 
    {
      // A planar loft (all profiles coplanar) has no depth to cap; capping would
      // stitch coincident caps into a degenerate zero-volume "solid". Reject it
      // (cap_ends=FALSE still yields the open sheet); non-coplanar profiles pass.
      SmPoint3d  sPt0;
      SmVector3d sN0;
      SmSurface *pSrf0     = sFaces[0]->GetSurface();
      SmBoolean  bCoplanar = (pSrf0 != NULL && pSrf0->IsPlanar(dTol, &sPt0, &sN0));
      const double dCoplanarTol = smos_Max(dTol, bAppoxTol3d);
      for (ULONG ii=1; bCoplanar && ii<sFaces.GetSize(); ii++)
        {
          SmPoint3d  sPt;
          SmVector3d sN;
          SmSurface *pSrf = sFaces[ii]->GetSurface();
          bCoplanar = pSrf && pSrf->IsPlanar(dTol, &sPt, &sN) && sN0.IsParallelTo(sN)
                   && smos_Fabs(sN0.Dot(SmVector3d(sPt.x-sPt0.x, sPt.y-sPt0.y, sPt.z-sPt0.z))) <= dCoplanarTol;
        }
      AERN_MSG(!bCoplanar, SM_ERR_INVALID_INPUT,
               _T("CreateSkinPrimitive: cannot cap a planar loft (coplanar profiles); use cap_ends=FALSE."));

      SmTArray<SmCurve*> sStartCurves;
      SmTArray<SmCurve*> sEndCurves;

      // A skin may be split into several U spans at intermediate profiles,
      // especially when the loft degree is linear.  Those boundaries are loft
      // stations, not transverse faces: only side faces touching the full
      // skin's U endpoints bound the caps.
      if(rSideFaces.GetSize() == 0)
        { SER(SM_ERR); }

      double dSkinUMin =  SM_BIG_DOUBLE;
      double dSkinUMax = -SM_BIG_DOUBLE;
      const double dEndParamTol = SM_EFF_ZERO_SQRT; // Same boundary tolerance used when splitting side surfaces.
      for(ULONG ii=0; ii<rSideFaces.GetSize(); ii++)
        {
          SmFace *pSideFace = rSideFaces[ii];
          NER(pSideFace);
          SmSurface *pSurface = pSideFace->GetSurface();
          NER(pSurface);
          SmExtent2d sDomain = pSurface->GetNaturalUVDomain();
          dSkinUMin = smos_Min(dSkinUMin,sDomain.GetUMin());
          dSkinUMax = smos_Max(dSkinUMax,sDomain.GetUMax());
        }

      for(ULONG ii=0; ii<rSideFaces.GetSize(); ii++)
        {
          SmSurface *pSurface = rSideFaces[ii]->GetSurface();
          SmExtent2d sDomain = pSurface->GetNaturalUVDomain();

          if(smos_Fabs(sDomain.GetUMin()-dSkinUMin) <= dEndParamTol)
            {
              SmBSplineCurve *pStartCurve = NULL;
              SER(pSurface->CreateIsoParametricCurve(*pContext,SM_SP_U,
                      sDomain.GetUMin(),bAppoxTol3d/10.0,pStartCurve));
              sStartCurves.Add(pStartCurve);
            }

          if(smos_Fabs(sDomain.GetUMax()-dSkinUMax) <= dEndParamTol)
            {
              SmBSplineCurve *pEndCurve = NULL;
              SER(pSurface->CreateIsoParametricCurve(*pContext,SM_SP_U,
                      sDomain.GetUMax(),bAppoxTol3d/10.0,pEndCurve));
              sEndCurves.Add(pEndCurve);
            }
        }

      if(sStartCurves.GetSize() == 0 || sEndCurves.GetSize() == 0)
        { SER(SM_ERR); }

      // from list of all top and bot curves - create needed planar cap faces
      SER(pBrep->CreatePlanarFacesWith3DCurves(pRegion,sStartCurves,bAppoxTol3d,rStartFaces));
      SER(pBrep->CreatePlanarFacesWith3DCurves(pRegion,sEndCurves,bAppoxTol3d,rEndFaces));

    } // end when making cap faces

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
      if (bDebugMe) 
        {
          pBrep->WriteToFile(_T("e:/WORK/Nmtlib4/max_bad_stitch.txt"), SM_ASCII, TRUE, FALSE, 0.0);
        }
#endif // SM_DEBUG_CODE

  // stitch all the faces into a solid
  ULONG lStitched, lLamina;
  double dMaxVGap = 0.0, dMaxEGap = 0.0;
  SER(pBrep->StitchFaces(bAppoxTol3d,lStitched,lLamina,dMaxVGap,dMaxEGap));
    
  // all done
  return SM_SUCCESS;

} // end sm_CreateSkinPrimitiveInternal

/*******************************************************************//**
PURPOSE: build loft faces between the loops on a sequence of faces
            or one loft face between a sequence of loft curves.

NOTES: 

  The input loft profiles are specified in 1 of 2 ways.
    1. As the loops in a set of ordered faces.  The faces are ordered in space
         expecting loft surfaces to run from the 1st through the 2nd and so on to the last.
         Each face must have the same number of loops. Every loop in a face
         correlates to the loops in the neighbor faces based on its position 
         in the array returned by the function SmFace->GetLoops().
         The output will contain one lofted surface that runs through each
         set of correllated loops.

    2. A sequence of curves to loft. The output will contain one lofted face
         starting at the first curve and running through the curves in order
         to the last curve.

  In both cases, the loft curves will end up being u iso-parameter curves in the output faces.

  When bAppoxTol3d is 0.0, the loft surfaces will interpolate the
       loft profiles.  
  When bAppoxTol3d is greater than 0.0, the loft surfaces will 
       approximate the loft profiles.  Sometimes an approximate loft will
       be smoother than an interpolating surface.

  lDegree is the loft degree, 1 for linear, 2 for parabolic, 3 for cubic.
       Typically 3 works well for general loft shapes.
***********************************************************************/
SmStatus SmPrimitiveCreation::CreateSkinFromFaces
 (SmTArray<SmFace*>         & rFaces,          // in : When given, create lofts between matching loops on a sequence of faces
  SmTArray<SmBSplineCurve*> & rCurvesToSkin,   // in : when rFaces is empty, a sequence of profiles to loft
                                               //      profile curves are always u isoparameter curves in the loft surfaces.
  double                      dAppoxTol3d,     // in : 0.0 = loft surface interpolates loft profiles, 
                                               //      else loft surface approximates loft profile.
  ULONG                       lDegree,         // in : degree in loft direction, 1=linear, 2=parabolic, 3=cubic
  SmTArray<SmFace*>         & rNewFaces)       // out: One lofted face for every set of matched loops from rFaces or
                                               //      One lofted face running through rCurvesToSkin
{
  // init SmPrimitiveCreation object
  InitEntityMaps();

  // init output
  rNewFaces.ReSet();
  
  // locals
  SmTArray<SmFace*>              sAllFaces;
  const SmContext              * pContext = m_pRegion->GetBrep()->GetContext();
  SmTArray<ULONG>                sLayer;          // index to match SkinCurves curves.
                                                  //   a unique index value for every loop in each rFace face, or
                                                  //   or 1 index for every loftCurve given in rCurvesToSkin.
  SmTArray<SmBSplineCurve*>      sSkinCurves;     // one joined SmBplineCurve for every loop in every rFaces Face
                                                  //   or one curve for every rCurvesToSkin curve
  SmObjsDelete<SmBSplineCurve*>  sCleanBSP(&sSkinCurves);
  SmTArray<SmCompositeCurve*>    sComposites;
  ULONG                          lNumLayers = 0;  // number of loops on each face when rFaces is given
                                                  // 1 when onlyrCurveToSkin are given

  // for every input face
  for (ULONG i=0; i<rFaces.GetSize(); i++) 
    {
      SmFace *pFace = rFaces[i];
      SmVector3d sPlaneZ;
      SER(pFace->GetSurface()->EvaluateNormal(pFace->GetUVDomain().Evaluate(0.5,0.5),TRUE,TRUE,sPlaneZ));

      // Create composite curves corresponding to the loops of this face.
      SER(pFace->CreateComposites(*pContext,sComposites));
      SmObjsDelete<SmCompositeCurve*> sCleanComp(&sComposites);
      
      // each face should have the same number of loops
      if      (i==0)                                { lNumLayers = sComposites.GetSize();   }
      else if (lNumLayers != sComposites.GetSize()) {
                                                      SER(SM_ERR);   
                                                    }

      // for every composite - add a joined BSplineCurve equivalent to the SkinCurves array
      //  mark every joinedCurve with a layer associated with its loop number
      for (ULONG j=0; j<sComposites.GetSize(); j++) 
        {
          SmCompositeCurve * pComp     = sComposites[j];
          SmBSplineCurve   * pCompNurb = NULL ;
          SER(pComp->MakeCompositeNurb(*pContext,pCompNurb));
          sSkinCurves.Add(pCompNurb);
          sLayer.Add(j);
        }
    } // end iter every input face
  
  // when no faces are given
  if (rFaces.GetSize() == 0) 
    {
      // use the input loft curves
      sCleanBSP.Clear();
      sSkinCurves.Append(rCurvesToSkin);
      lNumLayers = 1;
      for (ULONG ii=0; ii<sSkinCurves.GetSize(); ii++) 
        {
          sLayer.Add(0);
        }
    }

  // loft face locals
  SmTArray<SmBSplineCurve*> sCurves;
  SmTArray<SmFace*>         sNewFaces;

  // for every layer:  a layer for every loop in a face when rFaces is given (all faces have the same number of loops)
  //                   1 layer when rFaces is empty and rCurvesToSkin are used instead
  for (ULONG il=0; il<lNumLayers; il++) 
    {
      // init loop values
      sCurves.ReSet();

      // build an array of sCurves containing only those loft curves sequenced to be lofted
      for (ULONG jj=0; jj<sSkinCurves.GetSize(); jj++) 
        {
          // when the loft curve is from the current layer
          if (il == sLayer[jj]) 
            { 
              // save it in the sCurves array
              sCurves.Add(sSkinCurves[jj]);   
            }
        } //end building this sCurves array

      // Skinned Surface Locals
      SmBSplineSurface * pNewBSS = NULL ;
      SmBSplineSurface * pDerivSurf[2];
      pDerivSurf[0] = NULL;
      pDerivSurf[1] = NULL;

      // build a skinned surface through the loft curves on this 'layer'
      SER(SmBSplineSurface::CreateSkinnedSurface(*pContext,               // in : context for new object construction
                                                 sCurves,FALSE,           // in : ordered cross section curves, FALSE = curves not synchronized
                                                 SM_SP_U,                 // in : cross-section curves are U iso-parameter curves
                                                 dAppoxTol3d,             // in : 0.0 = interpolate cross-section curves, else approx to tolerance
                                                 NULL,NULL,FALSE,NULL,    // in : no input rail curves, No Spine curve, No Spine param values
                                                 pDerivSurf,              // in : no start/stop cross boundary deriviatives specified
                                                 pNewBSS,                 // out: Skin Surface
                                                 lDegree));               // in : opt sweep direction degree, default:[3]
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
          if (bDebugMe) 
            {
              smgfx_Erase();
              smgfx_SetLook(1,2, 0,1,1);
              for (ULONG ll=0; ll<sCurves.GetSize(); ll++) 
                {
                  sCurves[ll]->Draw();
                }
              sm_GraphicsLoop();
              smgfx_SetLook(1,2, 0,0,1); pNewBSS->Draw(); sm_GraphicsLoop();
              sm_GraphicsLoop();
            }
#endif // SM_DEBUG_CODE
      // create a face from the natural domain of the new Skin Surface SmBSplineSurface
      m_pRegion->GetBrep()->CreateFacesFromSurface(pNewBSS,
                                                   pNewBSS->GetNaturalUVDomain(),
                                                   SM_CT_G1,
                                                   sNewFaces);
      rNewFaces.Append(sNewFaces);

    } // end iter every layer

  // all done
  return SM_SUCCESS;

} // end SmPrimitiveCreation::CreateSkinFromFaces

/*******************************************************************//**
PURPOSE: Create solid primtive by sweeping a closed profile along a path.

NOTES: 
***********************************************************************/
SmStatus SmPrimitiveCreation::CreatePipeSweep
 (double              dPipeRadius,         // in : 
  SmBSplineCurve    * pPathCurve,          // in : 
  double              bAppoxTol3d,         // NotUsed: in : 
  SmBoolean           bCapEnds,            // in : 
  SmBoolean           bTranslationalSweep, // in : 
  SmTArray<SmFace*> & rStartFaces,         // out: 
  SmTArray<SmFace*> & rSideFaces,          // out: 
  SmTArray<SmFace*> & rEndFaces)           // out: 
{
  SM_REF1(bAppoxTol3d) ;
  // empty entity maps - m_vMapToSame, m_vMapToHigher, m_vMapFromSame, and m_vMapFromLower
  InitEntityMaps();

  rStartFaces.ReSet();
  rSideFaces.ReSet();
  rEndFaces.ReSet();

  SmBrep* pBrep = m_pRegion->GetBrep();
  const SmContext *pContext = pBrep->GetContext();

  // Get the Shape from the first pipe
  SmAxis2Placement sPlace;
  SmCircle* pCircle = NULL;
  SmExtent1d sDomain( 0.0, 360.0 );
  SmCircle::CreateCanonical( *pContext, sPlace, dPipeRadius, pCircle, &sDomain );
      
  SmTArray<SmCurve*> sShapeCurves;
  sShapeCurves.Add(pCircle);
  
  SmExtent3d sBBox2;
  pPathCurve->CalculateBoundingBox(pPathCurve->GetNaturalInterval(),&sBBox2);
  double dSize = sBBox2.GetSize().Length();
  dSize = dSize/2.0;
  double dTolerance = dSize * SM_ZONE_TOL_3D;

  SmSweepOptions sOptions;
  sOptions.bCapEndsArg = bCapEnds;
  sOptions.bTranslationalSweep = bTranslationalSweep;
  sOptions.bMoveProfileCenterToPath = TRUE;
  sOptions.bOrientProfilePerpendicularToPath = TRUE;
  sOptions.bMoveToProfile = FALSE;

  SER(CreateCurveSweep(sShapeCurves,pPathCurve,NULL,NULL,dTolerance*100.0,&sOptions,
      rStartFaces,rSideFaces,rEndFaces));
  
  return SM_SUCCESS;
} // end SmPrimitiveCreation::CreatePipeSweep

/*******************************************************************//**
PURPOSE: Create a 4 face sphere Brep model without seams or poles.

NOTES: 
  This sphere has no poles (surface singularities)
              and no seams (closed surfaces).
  But it does have tolerances around the boundary of the
      cap faces.
***********************************************************************/
SmStatus SmPrimitiveCreation::CreateSphereNoPole
 (const SmContext * pContext,         // in : context for new object construction 
  double            dSphereRadius,    // in : desired sphere radius
  SmVector3d        sSphereCenter,    // in : desired sphere center                                
  double            dTolerance,       // in : max dist between distinct points (set less than Brep->Tol/2.0)
  SmBrep         *& pBrep)            // i/o: Empty Brep Target in which to build the Sphere 
{
  // This constructs a 4 face sphere without any poles or seams
  
  // locals
  SmFace                     * pFace1 = NULL ;
  SmFace                     * pFace2 = NULL ;
  SmFace                     * pFace3 = NULL ;
  SmFace                     * pFace4 = NULL ;
  SmTArray<SmEdge *>           sEdges1 ;
  SmTArray<SmEdge *>           sEdges2 ;
  SmTArray <SmCurve*>          s3DCurves ;
  SmTArray <SmBSplineCurve*>   sUVCurves, s2DCurves ;
  SmTArray <ULONG>             sLoops ;
  SmTArray <SmOrientType>      sCurveOrients ;
  SmTArray <SmPoint3d>         sLoopPoints ;
  SmRegion                   * pNewRegion = NULL ;
  SmShell                    * pNewShell  = NULL ;

  // Note: the tolerance passed should be at least as tight as Brep->Tol/2.0
  // double  dTol    = smos_Min(dTolerance, pBrep->GetTolerance()/2.0) ;
  double  dStitchTol3d = 10.0*dTolerance;
  
  NL_POINT sCenterRot, sX, sY; 
  NL_VECTOR   sAxisRot;
  NL_REAL     dAlpha;     // angle of rotation    
  NL_SURFACE  sur;
  NL_CURVE    curC;
  NL_STACKS   S;
  
  // First make some NLib circles and surfaces
  N_InitNurbs(&S);
  
  // circular arc from Radius * ( (.6, -.8, 0) through (1,0,0) to (.6,.8.0) )
  N_VectorCreate(0.0, 0.0, 0.0, &sCenterRot);
  N_VectorCreate(1.0, 0.0, 0.0, &sX);
  N_VectorCreate(0.0, 1.0, 0.0, &sY);
  double dStartAngRad = smos_ArcTangent( .8/.6 ) ;
  double dStartAngDeg = -SM_RAD2DEG(dStartAngRad) ;
  N_CrvInitArrays(&curC); 
     
  // make circular arc in the sX/sZ plane from -dHeight to +dHeight with dRadius
  N_CreateCircArc( sCenterRot, sX, sY,          // in : arc plane - orig, x, y
                   dSphereRadius,               // in : radius
                   dStartAngDeg, -dStartAngDeg, // in : arc start/stop angles in degrees
                   NL_QUADRATIC,                // in : Quadratic, Quartic or Quintic representation
                   &curC,                       // out: circular arc
                   &S) ;                        // in : memory stack
  
  // Sweep circular arc 180 degrees around the y axis to make center 1/2 surface   
  N_VectorCreate(0.0, 1.0, 0.0, &sAxisRot);
  dAlpha = 180.0;
  N_SrfInitArrays(&sur);
  N_CreateRevolvedSrf(&curC, sCenterRot, sAxisRot, dAlpha, NL_QUADRATIC, &sur, &S);
  
  // 1st Face: make sur into an SmBSplineSurface and then to an SmBrep 
  //     note: edges are natural boundaries and should be tolerance free
  SmBSplineSurface *pBSS0 = new(*pContext) SmBSplineSurface((gw_SURFACE *)&sur) ;
  pBrep->CreateFaceFromSurface(pBSS0, pBSS0->GetNaturalUVDomain(), pFace1);  
  if(pFace1) pFace1->GetEdges(sEdges1) ;

  // 2nd Face: rotate sur 180 degrees about y axis and make 2nd face (center 1/2 surface) in the SmBrep
  N_SrfRotateAtPt(&sur, sCenterRot, sAxisRot, dAlpha);
  SmBSplineSurface *pBSS1 = new(*pContext) SmBSplineSurface((gw_SURFACE *)&sur) ;
  pBrep->CreateFaceFromSurface(pBSS1, pBSS1->GetNaturalUVDomain(), pFace2);
  if(pFace2) pFace2->GetEdges(sEdges2) ;
  
  // 3rd Face: rotate sur 90 degrees about x axis - trim it to endCap circle
  N_VectorCreate(1.0, 0.0, 0.0, &sAxisRot);
  N_SrfRotateAtPt(&sur, sCenterRot, sAxisRot, -90.0);
  SmBSplineSurface *pBSS2 = new(*pContext) SmBSplineSurface((gw_SURFACE *)&sur) ;

  // 3rd face boundary: use Edge->Curves from previous sweeps to simplify inputs to upcoming Stitch() call
  //                    sEdges1[3]
  //                    sEdges2[3]
  
  // make sure order of curves has not changed
  SmPoint3d sTestPt1, sTestPt2 ;
  sEdges1[3]->GetCurve()->EvaluatePoint(sEdges1[3]->GetCurve()->GetNaturalInterval().Evaluate(0.5), sTestPt1) ; 
  sEdges2[3]->GetCurve()->EvaluatePoint(sEdges2[3]->GetCurve()->GetNaturalInterval().Evaluate(0.5), sTestPt2) ;

#ifdef SM_DEBUG_CODE
  // Note: upper and lower radius: rad  must be less than Height
  double dHeight  = 0.8*dSphereRadius ;  // pick dHeight such that: dHeight > sqrt(2.0)/2.0 * dSphereRadius; 
  SM_ASSERT_MSG(SM_IS_ZERO_TO_TOL(sTestPt1.y-dHeight,SmTol::GetScaledZero(sTestPt1)), _T("SmPrimitiveCreation::CreateSphereNoPole, AssumedFace->Edge order has changed - debug here")) ;  
  SM_ASSERT_MSG(SM_IS_ZERO_TO_TOL(sTestPt2.y-dHeight,SmTol::GetScaledZero(sTestPt2)), _T("SmPrimitiveCreation::CreateSphereNoPole, AssumedFace->Edge order has changed - debug here")) ;  
#endif // SM_DEBUG_CODE

  // Get 3rd face 3d boundary copies
  SmCurve * pCurveCopy1 = NULL, * pCurveCopy2 = NULL ;
  sEdges1[3]->GetCurve()->Copy(*pContext, pCurveCopy1) ;
  sEdges2[3]->GetCurve()->Copy(*pContext, pCurveCopy2) ;
  s3DCurves.Add(pCurveCopy1) ;
  s3DCurves.Add(pCurveCopy2) ;

  // Drop s3DCurves[0] Boundary to UVSpace
  double dMaxDistToSurf, dDeviation;
  pBSS2->DropCurve(* pContext,                            // in : context for new object construction                                        
                     pBSS2->GetNaturalUVDomain(),         // in : domain of interest for this surface                                        
                   * s3DCurves[0],                        // in : Curve to project onto the surface                                          
                     s3DCurves[0]->GetNaturalInterval(),  // in : interval of interest for target curve                                      
                     dTolerance,                          // in : max allowed distance between 3dCurve and Surface for successful drops.        
                     dMaxDistToSurf,                      // out: MaxDist(DropPt, 3dCurvePt), found by sampling, may be slightly less than actual max.                                                           
                     dDeviation,                          // out: MaxDist(DropPt->SrfNormalLine, 3dCurvePt), Quality Measure: expected to be close to 0.0     
                     s2DCurves);                          // out: 1 (or 2) curves constructed by projection.                                 
                                                          //      (2 curves for closed surfaces when cr3DCurve is coincident with seam)      
  // Expect 1 UVCurve - set MakeFaceWithCurves input arrays
  SM_ASSERT_MSG(s2DCurves.GetSize() == 1, _T("SmPrimitiveCreation::CreateSphereNoPole, Dropped Edge failed to generate 1 UVtrimCurve")) ;  
  SmEdgeuse    * pUpwardEdgeuse1 = sEdges1[3]->GetUpwardEdgeuseOfFace(pFace1) ; 
  SmOrientType   eOrientation1   = pUpwardEdgeuse1->GetOrientation() ; 
  sUVCurves.Add(s2DCurves[0]) ; 
  sCurveOrients.Add(eOrientation1 == SM_OT_SAME ? SM_OT_OPPOSITE : SM_OT_SAME);    // curve orientation opposite of the edge's use in pFace1

  // Drop s3DCurves[1] Boundary to UVSpace
  pBSS2->DropCurve(* pContext,                            // in : context for new object construction                                        
                     pBSS2->GetNaturalUVDomain(),         // in : domain of interest for this surface                                        
                   * s3DCurves[1],                        // in : Curve to project onto the surface                                          
                     s3DCurves[1]->GetNaturalInterval(),  // in : interval of interest for target curve                                      
                     dTolerance,                          // in : max allowed distance between 3dCurve and Surface for successful drops.        
                     dMaxDistToSurf,                      // out: MaxDist(DropPt, 3dCurvePt), found by sampling, may be slightly less than actual max.                                                           
                     dDeviation,                          // out: MaxDist(DropPt->SrfNormalLine, 3dCurvePt), Quality Measure: expected to be close to 0.0     
                     s2DCurves);                          // out: 1 (or 2) curves constructed by projection.                                 
                                                          //      (2 curves for closed surfaces when cr3DCurve is coincident with seam)      
  // Expect 1 UVCurve - set MakeFaceWithCurves input arrays
  SM_ASSERT_MSG(s2DCurves.GetSize() == 1, _T("SmPrimitiveCreation::CreateSphereNoPole, Dropped Edge failed to generate 1 UVtrimCurve")) ;  
  SmEdgeuse    * pUpwardEdgeuse2 = sEdges2[3]->GetUpwardEdgeuseOfFace(pFace2) ;
  SmOrientType   eOrientation2   = pUpwardEdgeuse2->GetOrientation() ; 
  sUVCurves.Add(s2DCurves[0]) ; 
  sCurveOrients.Add(eOrientation2 == SM_OT_SAME ? SM_OT_OPPOSITE : SM_OT_SAME);    // curve orientation same as loop

  // Loop has 2 edges
  sLoops.Add(2);                    // 2 curves in outer loop

  pBrep->MakeFaceWithCurves
     (pBrep->GetInfiniteRegion(),   // in : region to contain new topology objects
      sLoops,                       // in : 1 entry per loop, value = loop edge count, 1st entry=outer loop
      &s3DCurves,                   // in : opt ordered 3d trimming curves assigned to loops per sLoopEUCounts
      &sUVCurves,                   // in : opt ordered 2d trimming curves assigned to loops per sLoopEUCounts
      sCurveOrients,                // in : associated orients for each trimming curve, SM_OT_SAME or SM_OT_OPPOSITE
      sLoopPoints,                  // in : Point positions to build SmVertex VertexLoops
      pBSS2,                        // in : new face->Surface
      pBSS2->GetNaturalUVDomain(),  // in : domain of Surface used by face
      SM_OT_SAME,                   // in : Surface orient, oneof SM_OT_SAME or SM_OT_OPPOSITE                                    
      pNewRegion,                   // out: New region if any. NULL when building trimmed surfaces, may be NotNULL for solids.
      pNewShell,                    // out: New shell if any.  Trimmed surfaces always create a new shell.
      pFace3);                      // out: the new face

  // 4th face: rotate sur 180 degrees about x axis
  N_VectorCreate(1.0, 0.0, 0.0, &sAxisRot);
  N_SrfRotateAtPt(&sur, sCenterRot, sAxisRot, 180.0);
  SmBSplineSurface *pBSS3 = new(*pContext) SmBSplineSurface((gw_SURFACE *)&sur) ;


  // make sure order of curves has not changed
  sEdges1[1]->GetCurve()->EvaluatePoint(sEdges1[1]->GetCurve()->GetNaturalInterval().Evaluate(0.5), sTestPt1) ; 
  sEdges2[1]->GetCurve()->EvaluatePoint(sEdges2[1]->GetCurve()->GetNaturalInterval().Evaluate(0.5), sTestPt2) ;

  SM_ASSERT_MSG(SM_IS_ZERO_TO_TOL(sTestPt1.y+dHeight,SmTol::GetScaledZero(sTestPt1)), _T("SmPrimitiveCreation::CreateSphereNoPole, AssumedFace->Edge order has changed - debug here")) ;  
  SM_ASSERT_MSG(SM_IS_ZERO_TO_TOL(sTestPt2.y+dHeight,SmTol::GetScaledZero(sTestPt2)), _T("SmPrimitiveCreation::CreateSphereNoPole, AssumedFace->Edge order has changed - debug here")) ;  
  
  // Get 4th face 3d boundary curve copies
  s3DCurves.ReSet() ;
  sEdges1[1]->GetCurve()->Copy(*pContext, pCurveCopy1) ;
  sEdges2[1]->GetCurve()->Copy(*pContext, pCurveCopy2) ;
  s3DCurves.Add(pCurveCopy1) ;
  s3DCurves.Add(pCurveCopy2) ;

  // Drop s3DCurves[0] Boundary to UVSpace
  pBSS3->DropCurve(* pContext,                            // in : context for new object construction                                        
                     pBSS3->GetNaturalUVDomain(),         // in : domain of interest for this surface                                        
                   * s3DCurves[0],                        // in : Curve to project onto the surface                                          
                     s3DCurves[0]->GetNaturalInterval(),  // in : interval of interest for target curve                                      
                     dTolerance,                          // in : max allowed distance between 3dCurve and Surface for successful drops.        
                     dMaxDistToSurf,                      // out: MaxDist(DropPt, 3dCurvePt), found by sampling, may be slightly less than actual max.                                                           
                     dDeviation,                          // out: MaxDist(DropPt->SrfNormalLine, 3dCurvePt), Quality Measure: expected to be close to 0.0     
                     s2DCurves);                          // out: 1 (or 2) curves constructed by projection.                                 
                                                          //      (2 curves for closed surfaces when cr3DCurve is coincident with seam)      
  // Expect 1 UVCurve
  SM_ASSERT_MSG(s2DCurves.GetSize() == 1, _T("SmPrimitiveCreation::CreateSphereNoPole, Dropped Edge failed to generate 1 UVtrimCurve")) ;  
  sUVCurves.ReSet() ;
  sCurveOrients.ReSet() ;
  pUpwardEdgeuse1 = sEdges1[1]->GetUpwardEdgeuseOfFace(pFace1) ; 
  eOrientation1   = pUpwardEdgeuse1->GetOrientation() ; 
  sUVCurves.Add(s2DCurves[0]) ; 
  sCurveOrients.Add(eOrientation1 == SM_OT_SAME ? SM_OT_OPPOSITE : SM_OT_SAME);    // these curve orientations are opposite to loop

  // Drop s3DCurves[1] Boundary to UVSpace
  pBSS3->DropCurve(* pContext,                            // in : context for new object construction                                        
                     pBSS3->GetNaturalUVDomain(),         // in : domain of interest for this surface                                        
                   * s3DCurves[1],                        // in : Curve to project onto the surface                                          
                     s3DCurves[1]->GetNaturalInterval(),  // in : interval of interest for target curve                                      
                     dTolerance,                          // in : max allowed distance between 3dCurve and Surface for successful drops.        
                     dMaxDistToSurf,                      // out: MaxDist(DropPt, 3dCurvePt), found by sampling, may be slightly less than actual max.                                                           
                     dDeviation,                          // out: MaxDist(DropPt->SrfNormalLine, 3dCurvePt), Quality Measure: expected to be close to 0.0     
                     s2DCurves);                          // out: 1 (or 2) curves constructed by projection.                                 
                                                          //      (2 curves for closed surfaces when cr3DCurve is coincident with seam)      
  // Expect 1 UVCurve
  SM_ASSERT_MSG(s2DCurves.GetSize() == 1, _T("SmPrimitiveCreation::CreateSphereNoPole, Dropped Edge failed to generate 1 UVtrimCurve")) ;  
  pUpwardEdgeuse2 = sEdges2[1]->GetUpwardEdgeuseOfFace(pFace2) ; 
  eOrientation2   = pUpwardEdgeuse2->GetOrientation() ; 
  sUVCurves.Add(s2DCurves[0]) ; 
  sCurveOrients.Add(eOrientation2 == SM_OT_SAME ? SM_OT_OPPOSITE : SM_OT_SAME);    // these curve orientations are opposite to loop

  // Loop has 2 edges
  sLoops.ReSet() ;
  sLoops.Add(2);                    // 2 curves in outer loop

  pBrep->MakeFaceWithCurves
     (pBrep->GetInfiniteRegion(),   // in : region to contain new topology objects
      sLoops,                       // in : 1 entry per loop, value = loop edge count, 1st entry=outer loop
      &s3DCurves,                   // in : opt ordered 3d trimming curves assigned to loops per sLoopEUCounts
      &sUVCurves,                   // in : opt ordered 2d trimming curves assigned to loops per sLoopEUCounts
      sCurveOrients,                // in : associated orients for each trimming curve, SM_OT_SAME or SM_OT_OPPOSITE
      sLoopPoints,                  // in : Point positions to build SmVertex VertexLoops
      pBSS3,                        // in : new face->Surface
      pBSS3->GetNaturalUVDomain(),  // in : domain of Surface used by face
      SM_OT_SAME,                   // in : Surface orient, oneof SM_OT_SAME or SM_OT_OPPOSITE                                    
      pNewRegion,                   // out: New region if any. NULL when building trimmed surfaces, may be NotNULL for solids.
      pNewShell,                    // out: New shell if any.  Trimmed surfaces always create a new shell.
      pFace4);                      // out: the new face

#ifdef SM_DEBUG_CODE
ULONG di, dj ;
SmBoolean bDebugMe = FALSE ;
  // draw faces and dump Brep, Face, Edges, and edge->Curves
  if(bDebugMe)
    {
      SM_DUMP_AND_ASSERT_VALID(pBrep) ;  // won't pass - expect coincident geometry problems about to be fixed by Stitch()
      SM_DUMP_AND_ASSERT_VALID(pFace1) ; 
      SM_DUMP_AND_ASSERT_VALID(pFace2) ; 
      SM_DUMP_AND_ASSERT_VALID(pFace3) ; 
      SM_DUMP_AND_ASSERT_VALID(pFace4) ; 

      SmSurface * pSurface1 = pFace1 ? pFace1->GetSurface() : NULL ;
      SmSurface * pSurface2 = pFace2 ? pFace2->GetSurface() : NULL ;
      SmSurface * pSurface3 = pFace3 ? pFace3->GetSurface() : NULL ;
      SmSurface * pSurface4 = pFace4 ? pFace4->GetSurface() : NULL ;

      SmTArray<SmEdge *> sEdges[4] ;

      if(pFace1) pFace1->GetEdges(sEdges[0]) ;
      if(pFace2) pFace2->GetEdges(sEdges[1]) ;
      if(pFace3) pFace3->GetEdges(sEdges[2]) ;
      if(pFace4) pFace4->GetEdges(sEdges[3]) ;

      for(di=0;di<4;di++)
        { for(dj=0;dj<sEdges[di].GetSize();dj++) { sEdges[di][dj]->Dump() ; 
                                                   sEdges[di][dj]->GetCurve()->Dump() ; 
                                                 }
        }

      smgfx_Erase() ;
      smgfx_SetLook(1,4, 0,0,1) ; if(pBrep)       pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,4, 0,1,1) ; if(pSurface1)   pSurface1->DrawUV(7,7) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,4, 0,1,1) ; if(pSurface2)   pSurface2->DrawUV(7,7) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,4, 0,1,1) ; if(pSurface3)   pSurface3->DrawUV(7,7) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,4, 0,1,1) ; if(pSurface4)   pSurface4->DrawUV(7,7) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,6, 0,0,1) ; if(sEdges1[3])  sEdges1[3]->GetCurve()->DrawParams() ;sm_GraphicsLoop() ; 
      smgfx_SetLook(1,6, 0,0,1) ; if(sEdges2[3])  sEdges2[3]->GetCurve()->DrawParams() ;sm_GraphicsLoop() ;
      smgfx_SetLook(1,6, 0,0,1) ; if(sEdges1[1])  sEdges1[1]->GetCurve()->DrawParams() ;sm_GraphicsLoop() ; 
      smgfx_SetLook(1,6, 0,0,1) ; if(sEdges2[1])  sEdges2[1]->GetCurve()->DrawParams() ;sm_GraphicsLoop() ;
      smgfx_SetLook(1,4, 0,0,0) ; if(pFace1)      pFace1->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,4, 0,0,0) ; if(pFace2)      pFace2->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,4, 0,0,0) ; if(pFace3)      pFace3->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,4, 0,0,0) ; if(pFace4)      pFace4->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop() ;
      for(di=0;di<4;di++)
        { for(dj=0;dj<sEdges[di].GetSize();dj++) { SmEdge * pEdge  = sEdges[di][dj] ;
                                                   SmCurve *pCurve = pEdge ? pEdge->GetCurve() : NULL ;
                                                   smgfx_SetLook(3,4, 1,0,0) ; if(pCurve) pCurve->DrawParams() ; sm_GraphicsLoop() ;
                                                   smgfx_SetLook(5,6, 1,0,1) ; if(pEdge)  pEdge->Draw() ; sm_GraphicsLoop() ;
                                                 }
        }
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE  

  // Stitch the Brep into a manifold solid
  ULONG  lNumStitched, lNumLam ;
  double dMaxVGap = 0.0, dMaxEGap = 0.0;
  SmStatus sRtn = pBrep->StitchFaces(dStitchTol3d, lNumStitched, lNumLam, dMaxVGap, dMaxEGap);

#ifdef SM_DEBUG_CODE
  // Dump the stitched result
  if(bDebugMe)
    {
      SmSphere         * pTestSphere ;
      SmAxis2Placement   sTestOrigin ;
      SmSphere::CreateCanonical(*pContext, sTestOrigin, dSphereRadius, pTestSphere) ;
      SmObjDelete        sTestClean(pTestSphere) ;

      SM_DUMP_AND_ASSERT_VALID(pBrep) ;  // expect no problems

      smgfx_Erase() ;
      smgfx_SetLook(1,4, 1,1,0) ; if(pTestSphere) pTestSphere->DrawUV() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,4) ;        if(pBrep)       pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,4) ;        if(pBrep)       pBrep->DrawUV(TRUE) ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE
 
  // move the sphere to requested location
  SmAxis2Placement sRef;
  sRef.Translate(sSphereCenter);
  pBrep->Transform(sRef);

  // all done
  return sRtn ;

}  // end SmPrimitiveCreation::CreateSphereNoPole

/*******************************************************************//**
PURPOSE: Creates a conic solid with elliptic ends and endcaps (opt)
      
NOTES: Cone is built with the height as the z axis from
       the origin, and conic in the XY plane.
       The ends can be tilted about the XY axis, making the ends 
       elliptical

***********************************************************************/
SmStatus SmPrimitiveCreation::CreateConeEllipticEnds
 (double front_radius,     // in : applies to cone radius at origin of endcap (before tilt)
  double rear_radius,      // in : 
  double height,           // in : measured from the center(origin) of each end-ellipse
  double front_Y_tilt,     // in : tilt about the X axis in degrees
  double rear_Y_tilt,      // in : 
  double front_X_tilt,     // in : tilt about the Y axis in degrees
  double rear_X_tilt,      // in : 
  SmBoolean bEndcaps,      // in : if TRUE, close off end caps
  SmBrep *& pBrep)         // out: external pBrep = new (*pContext) SmBrep;
{
  NL_STACKS SL;
  N_InitNurbs(&SL);
    
  NL_FLAG error = 0;
  // NL_FLAG dir = SM_SP_V + 1;
  NL_SURFACE cone_sur;
  N_SrfInitArrays(&cone_sur);
       
  // makes linear direction is V, conic direction is U
  error = N_ConeEllipticEnds(front_radius, rear_radius, height,
      front_Y_tilt, rear_Y_tilt,
      front_X_tilt, rear_X_tilt,
      &cone_sur, 
      &SL);
  if ( error != 0 ) return (SM_ERR);
    
  // convert the surface into a Brep
  const SmContext *pContext = pBrep->GetContext();
  SmBSplineSurface *pConeSrf = new(*pContext) SmBSplineSurface(&cone_sur) ;
  N_EndNurbs(&SL);
    
  SmFace *pConeFace = NULL;
  SER(pBrep->CreateFaceFromSurface(pConeSrf, pConeSrf->GetNaturalUVDomain(), pConeFace));

  if (!bEndcaps) return (SM_SUCCESS);
    
  // now build a trimmed plane for each end    
  SmBSplineCurve *pFrontEllipse=NULL, *pRearEllipse=NULL;
    
  // U direction is for rotation ( circle or ellipse)
  SER(pConeSrf->CreateIsoParametricCurve(*pContext, SM_SP_U, 0.0, 0.0, pFrontEllipse));
  SER(pConeSrf->CreateIsoParametricCurve(*pContext, SM_SP_U, 1.0, 0.0, pRearEllipse));
    
  // to get outward facing face, reverse.
  SmExtent1d sIvl = pFrontEllipse->GetNaturalInterval();
  pFrontEllipse->ReverseParameterization(sIvl, sIvl);

  // make face from front
  SmTArray < SmCurve *> pCurves;
  SmFace *pFaceF = NULL;        
  pCurves.Add(pFrontEllipse);    
    
  // Create Face with endcap curve
  SER(pBrep->CreatePlanarFaceWith3DCurves(pBrep->GetInfiniteRegion(), // in : region to contain newFace - not checked
                                                                      //      NULL for infinite region.
                                          pCurves,                    // in : array of planar curves to bound newFace - curves used by NewEdges
                                          0.001,                      // in : 3d Distance for planarity checks
                                          pFaceF));                   // out: new face
    
  SmFace *pFaceR = NULL;
  pCurves.ReSet();       
  pCurves.Add(pRearEllipse);    
    
  // Create Face from rear
  SER(pBrep->CreatePlanarFaceWith3DCurves(pBrep->GetInfiniteRegion(), // in : region to contain newFace - not checked
                                                                      //      NULL for infinite region.
                                          pCurves,                    // in : array of planar curves to bound newFace - curves used by NewEdges
                                          0.001,                      // in : 3d Distance for planarity checks
                                          pFaceR));                   // out: new face
        
  // SER(pBrep->ShrinkGeometry()); //it's ok to leave plane slightly bigger than end-cap

  // Stitch all faces together   
  SER(pBrep->StitchAndOrient());

       
  return SM_SUCCESS;

} // end SmPrimitiveCreation::CreateConeEllipticEnds

/*******************************************************************//**
PURPOSE: Create a smooth blended surface between two edges. 

NOTES: This version of CreateBlendPrimitive has controls for the blend
  as explicit argument values. The other overloaded version has them in the
  SmDerivSurfDef. 
  
  Use eCurvatureType to control the curvature at the edge.
  Use eBlendEndType to control whether the edge between the new and original
  surfaces are smooth or cusped.
  dStartTanLength and dEndTanLength control the corresponding take-off vectors

  Edges must travel in same general direction to avoid a twisted surface.
  Set lCurvesDirFlag to 0 for an guess based on the geometry of the
  edges being connected. Set to 1 or 2 to force certain behavior as noted 
  in the argument list.

DEVELOPMENT NOTES ---- This creation method could be augmented to add a method
  that works with geometry only, without using or creating
  any topology. For example, a higher level function might take
  a surface and a curve for each side. The function would drop the
  curve onto the surface to create the face and edge and then call
  the SmPrimitiveCreation::CreateBlendPrimitive(). Alternatively,
  we could blend from a curve on surface instead of an edge.

***********************************************************************/
SmStatus SmPrimitiveCreation::CreateBlendPrimitive
 (SmEdgeuse       * pEU1,            // in : target start blend surface 
  SmEdgeuse       * pEU2,            // in : start surface boundary curve
  SmFace         *& rpBlendFace,     // out: 
  SmCurvatureType   eCurvatureType,  // in : blend surface continuity
  SmBlendEndType    eBlendEndType,   // in : blend ends smooth or cusped
  ULONG             lCurvesDirFlag,  // in : 0 = Guess curve directions based on geometry
                                     //      1 = run blend between start of Edge1->Curve to start of Edge2->Curve
                                     //      2 = run blend between start of edge1->Curve to end of Edge2->Curve 
  double            dStartTanLength, // in : Take-off vector scaling at start of EU1->Curve
  double            dEndTanLength)   // in : Take-off vector scaling at end of EU1->Curve
{
  // init state                    
  InitEntityMaps() ;

  // context and brep locals
  SmBrep          * pBrep = m_pRegion->GetBrep();
  const SmContext * pContext = pBrep->GetContext();

  // extract an lBlendFlag value
  ULONG lBlendFlag = eCurvatureType == SM_CT_NO_CURVATURE ? 1
    : eCurvatureType == SM_CT_G2_FROM_SURFACE ? 2
    : eCurvatureType == SM_CT_G3_FROM_SURFACE ? 3
    : 2 ;

  // Create SmDerivSurfDef with user inputs
  SmDerivSurfDefinition sDSDef;
  sDSDef.m_eCurvatureType  = eCurvatureType;
  sDSDef.m_eBlendEndType   = eBlendEndType;
  sDSDef.m_dStartTanLength = dStartTanLength;
  sDSDef.m_dEndTanScale    = dEndTanLength / dStartTanLength;
  
  // pass the call along
  SmSurface *pBlendSurface = NULL ;
  SER_MSG(SmBrep::CreateEdgeEdgeBlend( *pContext, lBlendFlag,
                                         pEU1, pEU2,
                                         lCurvesDirFlag, &sDSDef,
                                         pBlendSurface),
          _T("Blend Surface Construction Failure")) ;

  //      // face locals
  //      SmSurface *pSurface1 = pEdgeuse1->GetFace()->GetSurface() ;
  //      SmSurface *pSurface2 = pEdgeuse2->GetFace()->GetSurface() ;
  //      
  //      // edge locals
  //      SmExtent1d       sIvl1       = pEdge1->GetInterval();
  //      SmExtent1d       sIvl2       = pEdge2->GetInterval();
  //      SmBSplineCurve * pEdgeCurve1 = (SmBSplineCurve*)pEdgeuse1->GetEdge()->GetCurve();
  //      SmBSplineCurve * pEdgeCurve2 = (SmBSplineCurve*)pEdgeuse2->GetEdge()->GetCurve();
  //      SmBSplineCurve   sCurve1(*pEdgeCurve1) ; 
  //      SmBSplineCurve   sCurve2(*pEdgeCurve2) ;
  //      double dTolerance = smos_Max(pEdge1->GetTolerance(), pEdge2->GetTolerance()) ;
  //      
  //      // make sure two curve intervals run in the same direction and share a common parameterization
  //      if(bSameDirCurves == FALSE) { sCurve2.ReverseParameterization(sIvl2, sIvl2) ; }
  //      if(!sIvl1.AreEqual(sIvl2))
  //        {
  //          // rescale Curve2
  //          double dScale  = (sIvl1.GetMin() - sIvl1.GetMax()) / (sIvl2.GetMin() - sIvl2.GetMax()) ;
  //          double dOffset = (sIvl2.GetMin() * sIvl1.GetMax()) - (sIvl1.GetMin() * sIvl2.GetMax()) / (sIvl2.GetMin() - sIvl2.GetMax()) ;
  //      
  //          SmExtent1d sNewIvl = sCurve2.GetNaturalInterval() ;
  //          sNewIvl.SetMinMax(sNewIvl.GetMin() * dScale + dOffset,
  //                            sNewIvl.GetMax() * dScale + dOffset) ;
  //          sCurve2.EditParameterization(sNewIvl, FALSE) ;
  //          sIvl2 = sIvl1 ;
  //      
  //        } // end need to reparameterize Curve2 check
  //      
  //      // pick the bIntoFace values for curve/Surface pairs
  //      // bOffToTheRight: TRUE = start blend towards the outside of face1 from edge1
  //      //                 FALSE= start blend towards the inside of face1 from edge1
  //      // bInFromTheLeft: TRUE = end blend from the outside of face2 into edge2
  //      //                 FALSE= end blend from the inside of face2 into edge2
  //      SmVector3d sEdge1Pos,      sEdge2Pos ;
  //      SmVector3d sEdge1Tangent,  sEdge2Tangent ;
  //      SmVector3d sEdge1Binormal, sEdge2Binormal ;
  //      SmVector3d sFace1Normal,   sFace2Normal ;
  //      pEdgeuse1->EvaluateBinormal(sIvl1.Evaluate(.5), TRUE, sEdge1Pos, sEdge1Binormal, &sEdge1Tangent, &sFace1Normal) ; 
  //      pEdgeuse2->EvaluateBinormal(sIvl2.Evaluate(.5), TRUE, sEdge2Pos, sEdge2Binormal, &sEdge2Tangent, &sFace2Normal) ; 
  //      if(pEdgeuse1->GetOrientation()               == SM_OT_OPPOSITE) sEdge1Tangent = -sEdge1Tangent ;
  //      if(pEdgeuse2->GetOrientation()               == SM_OT_OPPOSITE) sEdge2Tangent = -sEdge2Tangent ;
  //      if(pEdgeuse1->GetFaceuse()->GetOrientation() == SM_OT_OPPOSITE) sFace1Normal = -sFace1Normal ;
  //      if(pEdgeuse2->GetFaceuse()->GetOrientation() == SM_OT_OPPOSITE) sFace2Normal = -sFace2Normal ;
  //      
  //      // set edge1 blend direction to run out of the face
  //      // set edge2 blend direction to run into the face
  //      SmVector3d sCurve1Left = sFace1Normal * sEdge1Tangent ;
  //      SmVector3d sCurve2Left = sFace2Normal * sEdge2Tangent ;
  //      SmBoolean bOffToTheRight = sEdge1Binormal.Dot(sCurve1Left) >= 0.0 ? TRUE : FALSE ;
  //      SmBoolean bInFromTheLeft = sEdge2Binormal.Dot(sCurve2Left) >= 0.0 ? FALSE : TRUE ;
  //      
  //      #ifdef SM_DEBUG_CODE
  //      SmBoolean bDebugMe = FALSE ;
  //      // draw 
  //      if(bDebugMe)
  //        {
  //          pFace1->Dump() ;
  //          pFace2->Dump() ;
  //          SM_ASSERT_VALID(pFace1) ;
  //          SM_ASSERT_VALID(pFace2) ;
  //              
  //          smgfx_Erase() ;
  //          smgfx_SetLook(1,2, 0,0,1) ; if(pFace1) pFace1->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop() ;
  //          smgfx_SetLook(1,2, 0,1,0) ; if(pFace2) pFace2->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop() ;
  //          smgfx_SetLook(1,2, 0,0,1) ; if(pSurface1) pSurface1->Draw(TRUE) ; sm_GraphicsLoop() ;
  //          smgfx_SetLook(1,2, 0,1,0) ; if(pSurface2) pSurface2->Draw(TRUE) ; sm_GraphicsLoop() ;
  //          smgfx_SetLook(1,2, 0,0,1) ; if(pSurface1) pSurface1->DrawVectorField(pSurface1->GetNaturalUVDomain(),3,3,SM_DM_UNIT_NORMAL) ; sm_GraphicsLoop() ;
  //          smgfx_SetLook(1,2, 0,1,0) ; if(pSurface2) pSurface2->DrawVectorField(pSurface2->GetNaturalUVDomain(),3,3,SM_DM_UNIT_NORMAL) ; sm_GraphicsLoop() ;
  //      
  //          smgfx_SetLook(2,3, 0,1,1) ; if(pEdge1) pEdge1->Draw() ; sm_GraphicsLoop() ;
  //          smgfx_SetLook(2,3, 0,1,1) ; if(pEdge2) pEdge2->Draw() ; sm_GraphicsLoop() ;
  //          smgfx_SetLook(3,4, 1,0,1) ; sCurve1.Draw(NULL,TRUE) ; sm_GraphicsLoop() ;
  //          smgfx_SetLook(3,4, 1,0,0) ; sCurve2.Draw(NULL,TRUE) ; sm_GraphicsLoop() ;
  //          smgfx_SetLook(3,4, 1,0,1) ; sCurve1.DrawParams() ; sm_GraphicsLoop() ;
  //          smgfx_SetLook(3,4, 1,0,0) ; sCurve2.DrawParams() ; sm_GraphicsLoop() ;
  //          smgfx_SetLook(2,3, 1,0,0) ; sEdge1Binormal.Draw(&sEdge1Pos) ; sm_GraphicsLoop() ;
  //          smgfx_SetLook(2,3, 1,0,0) ; sEdge2Binormal.Draw(&sEdge2Pos) ; sm_GraphicsLoop() ;
  //          smgfx_SetLook(2,3, 0,1,1) ; sEdge1Tangent.Draw(&sEdge1Pos) ; sm_GraphicsLoop() ;
  //          smgfx_SetLook(2,3, 0,1,1) ; sEdge2Tangent.Draw(&sEdge2Pos) ; sm_GraphicsLoop() ;
  //          smgfx_SetLook(2,3, 1,0,1) ; sFace1Normal.Draw(&sEdge1Pos) ; sm_GraphicsLoop() ;
  //          smgfx_SetLook(2,3, 1,0,1) ; sFace2Normal.Draw(&sEdge2Pos) ; sm_GraphicsLoop() ;
  //          smgfx_SetLook(2,3, 0,1,0) ; sCurve1Left.Draw(&sEdge1Pos) ; sm_GraphicsLoop() ;
  //          smgfx_SetLook(2,3, 0,1,0) ; sCurve2Left.Draw(&sEdge2Pos) ; sm_GraphicsLoop() ;
  //          sm_GraphicsLoop() ;
  //        }
  //      #endif // SM_DEBUG_CODE
  //      
  //      // build blend surface
  //      SmSurface *pBlendSurface = NULL ;
  //      SER_MSG(pSurface1->CreateBlendSurface(*pContext, sCurve1, sIvl1, bOffToTheRight, 
  //                                            *pSurface2, sCurve2, sIvl2, bInFromTheLeft, 
  //                                            dTolerance, pOptDSDef, pBlendSurface),
  //              _T("SmSurface::CreateBlendSurface Failed")) ;

  // set brep for editing
  pBrep->m_bEditingEnabled = TRUE;

  // set side-effect - use new blend surface to build a face in this->pBrep
  pBrep->CreateFaceFromSurface(pBlendSurface, pBlendSurface->GetNaturalUVDomain(), rpBlendFace);

  // is there a need for a stitch here?

#ifdef SM_DEBUG_CODE
  SmBoolean bDebugMe = FALSE ;
  if (bDebugMe)
    {
    pBrep->Dump() ;

    ULONG lNumDerivs = sDSDef.GetNumDerivs() ;

    // test G1/G2/G3 continuity across blend
    SmContinuityType eContType1 = pEU1->GetSectorContinuity() ;
    SmContinuityType eContType2 = pEU2->GetSectorContinuity() ;

    SM_ASSERT_MSG(SM_IS_G1(eContType1), _T("Start Blend not G1")) ;
    SM_ASSERT_MSG(lNumDerivs < 2 || SM_IS_G2(eContType1), _T("Start Blend not G2")) ;
    SM_ASSERT_MSG(lNumDerivs < 3 || SM_IS_G3(eContType1), _T("Start Blend not G3")) ;

    SM_ASSERT_MSG(SM_IS_G1(eContType2), _T("End Blend not G1")) ;
    SM_ASSERT_MSG(lNumDerivs < 2 || SM_IS_G2(eContType2), _T("End Blend not G2")) ;
    SM_ASSERT_MSG(lNumDerivs < 3 || SM_IS_G3(eContType2), _T("End Blend not G3")) ;
    }
#endif // SM_DEBUG_CODE

  // all done
  return SM_SUCCESS;

} // end SmPrimitiveCreation::CreateBlendPrimitive

/*******************************************************************//**
PURPOSE: Create a smooth blended surface between two edges. 

NOTES: Controls for blending are in SmDerivSurfDefinition argument.
  See overloaded CreateBlendPrimitive for controls directly in the method signature.

  Use CurvatureType to control the curvature at the edge.
  Use BlendEndType to control whether the new surface is smooth or cusped
  where joined to the original surfaces.
  Edges must travel in same general direction to avoid a twisted surface.
  This behavior is controlled with bSameDirCurves

DEVELOPMENT NOTES ---- This creation method could be augmented to add a method
  that works with geometry only, without using or creating
  any topology. For example, a higher level function might take
  a surface and a curve for each side. The function would drop the
  curve onto the surface to create the face and edge and then call
  the SmPrimitiveCreation::CreateBlendPrimitive(). Alternatively,
  we could blend from a curve on surface instead of an edge.

***********************************************************************/
SmStatus SmPrimitiveCreation::CreateBlendPrimitive
 (SmFace                 * pFace1,          // in : target start blend surface 
  SmEdge                 * pEdge1,          // in : start surface boundary curve
  SmFace                 * pFace2,          // in : target end blend surface
  SmEdge                 * pEdge2,          // in : end surface boundary curve
  SmBoolean                bSameDirCurves,  // in : TRUE = run blend between start of Edge1->Curve to start of Edge2->Curve
                                            //      FALSE= run blend between start of edge1->Curve to end of Edge2->Curve 
  SmDerivSurfDefinition  * pOptDSDef,       // in : optional blend options, NULL to ignore
                                            //      NULL = use values hardcoded into this function
                                            //      hardcode's G2 blends
  SmFace                *& rpBlendFace)     // out: 
{
  // Get edgeuse connecting edge to face
  SmEdgeuse* pEdgeuse1 = pEdge1->GetEdgeuseOfFace( pFace1 ); 
  SmEdgeuse* pEdgeuse2 = pEdge2->GetEdgeuseOfFace( pFace2 );

  if(pEdgeuse1 == NULL || pEdgeuse2 == NULL) return SM_ERR_INVALID_INPUT;

  // extract values from pOptDSDef
  SmCurvatureType eCurvatureType  = pOptDSDef == NULL ? SM_CT_G2_FROM_SURFACE : pOptDSDef->m_eCurvatureType ;
  SmBlendEndType  eBlendEndType   = pOptDSDef == NULL ? SM_BE_NO_CUSP : pOptDSDef->m_eBlendEndType ;
  double          dStartTanLength = pOptDSDef == NULL ? 1.0 : pOptDSDef->m_dStartTanLength ;
  double          dEndTanLength   = pOptDSDef == NULL ? 1.0 : pOptDSDef->m_dEndTanScale * dStartTanLength ;
  ULONG           lCurvesDirFlag  = bSameDirCurves == TRUE ? 1 : 2 ;

  // Pass call along
  CreateBlendPrimitive(pEdgeuse1, 
                       pEdgeuse2, 
                       rpBlendFace,
                       eCurvatureType, 
                       eBlendEndType, 
                       lCurvesDirFlag,
                       dStartTanLength,
                       dEndTanLength);
  
  // all done
  return SM_SUCCESS;

} // end SmPrimitiveCreation::CreateBlendPrimitive

/*******************************************************************//**
PURPOSE: Perform a 2D Boolean operation on two Breps who have one
    or more planar faces.  

NOTES: Note that the planar surfaces from each Brep 
    must lie on the same infinite plane.  Each Brep must have only one
    surface and it must be a plane. 
    
    Modifies the pBrepA Brep in place and returns that pointer value
    as the output in rpResult.  Deletes the pBrepB pointer.

METHOD ---
   
   increments unlocked mark value  
***********************************************************************/
SmStatus SmPrimitiveCreation::Boolean2D
 (SmBrep                   * pBrepA,       // in : A of A operator B, returned as rpResult
  SmBrep                   * pBrepB,       // in : B or A operator B, deleted by this operation
  Sm2DBooleanOperationType   eBooleanType, // in : oneof: SM_2D_UNION,        = Union of 2D regions A and B       
                                           //             SM_2D_INTERSECTION, = Intersection of 2D regions A and B
                                           //             SM_2D_DIFFERENCE,   = Difference - A minus B  
                                           //             SM_2D_EXCLUSIVE_OR, = XOR = (A union B) - (A intersect B)          
                                           //             SM_2D_MERGE         = Merge Operation                   
  SmBrep                  *& rpResult)     // out: Modified pBrepA 
{
  // get global context
  const SmContext *pContext = pBrepA->GetContext();

  // construct A intersect B manager
  double dZoneTol3d   = smos_Max( pBrepA->GetTolerance(), pBrepB->GetTolerance() );
  double dApproxTol3d = SM_ZONE_TO_APPROXTOL3D(dZoneTol3d) ; 
  SmTopologyIntersector sTI( *pContext, pBrepA, pBrepB, dApproxTol3d, 20.0*SM_PI/180.0 );

  SmTemporaryChangeValue <SmBoolean> sTempChangeDB( ((SmContext*)pContext)->GetDoingBooleanRef(), TRUE );

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  // draw BrepA(blue) and BrepB(green)
  if (bDebugMe) 
    {
      SM_DUMP_AND_ASSERT_VALID(pBrepA) ;
      SM_DUMP_AND_ASSERT_VALID(pBrepB) ;

      smgfx_Erase();
      smgfx_SetLook(1,2, 0,0,1) ; pBrepA->Draw(TRUE); sm_GraphicsLoop();
      smgfx_SetLook(1,2, 0,1,0) ; pBrepB->Draw(TRUE); sm_GraphicsLoop() ;
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // turn on modifications in pBrepA and pBrepB
  SmTemporaryChangeValue<SmBoolean> sStack1(pBrepA->m_bEditingEnabled,TRUE);
  SmTemporaryChangeValue<SmBoolean> sStack2(pBrepB->m_bEditingEnabled,TRUE);

// Remove Composites
// //cbi_CEdge12:
// #ifdef SM_NO_COMPOSITES
//   pBrepA->m_bMakeComposites = FALSE;
//   pBrepB->m_bMakeComposites = FALSE;
// #else
//   pBrepA->m_bMakeComposites = TRUE;
//   pBrepB->m_bMakeComposites = TRUE;
// #endif // SM_NO_COMPOSITES
//
//  // The Brep used for a 2D Boolean must have just one surface
//  // and it must be a plane.  Note that there may be many composite
//  // faces on that one surface. 
  ULONG ii, jj ;
  SmTArray<SmFace*> sFaces ;
  SmTArray<SmFace*> sFacesA ;
  SmTArray<SmFace*> sFacesB ;
  pBrepA->GetFaces(sFacesA) ;
  pBrepB->GetFaces(sFacesB) ;

  // no work - no faces in pBrepA or pBrepB
  if(sFacesA.GetSize() == 0 || sFacesB.GetSize() == 0)
    { return SM_SUCCESS ; }

  // put all Faces in one list for convenience
  sFaces.Append(sFacesA) ;
  sFaces.Append(sFacesB) ;

  // locals for input checks
  SmVector3d  sPlanePoint1, sPlaneUnitNormal1 ; 
  SmVector3d  sPlanePoint2, sPlaneUnitNormal2 ;
  double      dDist, dDist1, dDist2 ;
  SmBoolean   bSuccess;
  SmFace    * pFace1    = sFaces[0] ;
  SmSurface * pSurface1 = pFace1->GetSurface() ;
  SmBoolean   bPlanar1  = pSurface1->IsPlanar(dZoneTol3d, &sPlanePoint1, &sPlaneUnitNormal1) ;

  // check state - planar Surface1
  AERN_MSG(bPlanar1 == TRUE, SM_ERR_INVALID_INPUT ,_T("SmPrimitiveCreation::Boolean2D - Bad input - pBrepA Face->Surface is not planar - both input Breps may only have planar coincident faces"))

  // check input - all surfaces of pBrepA and pBrepB are coincidnet planes
  for(ii=1;ii<sFaces.GetSize();ii++)
    {
      SmSurface * pSurface2 = sFaces[ii]->GetSurface() ;
      SmBoolean   bPlanar2  = pSurface2->IsPlanar(dZoneTol3d, &sPlanePoint2, &sPlaneUnitNormal2) ;

      // check state - planar Surface2
      AERN_MSG(bPlanar2 == TRUE, SM_ERR_INVALID_INPUT ,_T("SmPrimitiveCreation::Boolean2D - Bad input - Face->Surface is not planar - both input Breps may only have planar coincident faces"))

      // check state - coplanar
      smgu_PlanePointDistance(sPlanePoint1, sPlaneUnitNormal1, sPlanePoint2, dDist2) ;
      smgu_PlanePointDistance(sPlanePoint2, sPlaneUnitNormal2, sPlanePoint1, dDist1) ;
      AERN_MSG(   (smos_Max(dDist2, dDist1) < SM_ZONE_TO_XSECTTOL3D(dZoneTol3d)) 
               && (sPlaneUnitNormal1.IsParallelTo(sPlaneUnitNormal2)),  
               SM_ERR_INVALID_INPUT ,
               _T("SmPrimitiveCreation::Boolean2D - Bad input - Not all pBrebA and pBrepB are coincident - both input Breps may only have planar coincident faces")) ;

    } // end iter every Face making sure the Face->Surfaces are coplanar

  // extend pSurface1 domains to cover all other domains
  for(ii=1;ii<sFaces.GetSize();ii++)
    {
      SmFace    * pFace2           = sFaces[ii] ;
      SmSurface * pSurface2        = pFace2->GetSurface() ;
      SmExtent2d  sOtherFaceDomain = pFace2->GetUVDomain() ;

      SER( pSurface1->CoverCoincidentSurface( pSurface2,                         // in : other surface 
                                              SM_ZONE_TO_XSECTTOL3D(dZoneTol3d), // in : max separation allowed 
                                              bSuccess,                          // out: pOther will fit inside this; see notes above 
                                              dDist,                             // out: max distance found; see notes above 
                                              &sOtherFaceDomain) ) ;             // in : optional: use this subset of pOther 
    } // end growing pSurface1 domain to be big enough to cover all other face domains

  // extend the rest of the surface domains to cover the pSurface1 domain
  for(ii=1;ii<sFaces.GetSize();ii++)
    {
      SmFace    * pFace2           = sFaces[ii] ;
      SmSurface * pSurface2        = pFace2->GetSurface() ;
      SmExtent2d  sOtherFaceDomain = pFace2->GetUVDomain() ;

      SER( pSurface2->CoverCoincidentSurface( pSurface1,                         // in : other surface 
                                              SM_ZONE_TO_XSECTTOL3D(dZoneTol3d), // in : max separation allowed 
                                              bSuccess,                          // out: pOther will fit inside this; see notes above 
                                              dDist,                             // out: max distance found; see notes above 
                                              NULL) ) ;                          // in : optional: use this subset of pOther 
    } // end growing all other surfaces to be big enough to cover the pSurface1 domain

// Remove Composites
//  // check input - pBrepA has just one surface and it's a plane
//  if (sSurfaces.GetSize() != 1) { SER(SM_ERR); }
//  SmSurface *pSurfaceA = sSurfacesA[0];
//  SmPlane   *pPlaneA   = SM_CAST_PTR(SmPlane, pSurfaceA);
//  if (!pPlaneA) { SER(SM_ERR); } // Not a plane
//
//  // check input - pBrepB has just one surface and it's a plane
//  if (sSurfaces.GetSize() != 1) { SER(SM_ERR); }
//  SmSurface *pSurfaceB = sSurfacesB[0];
//  SmPlane *pPlaneB = SM_CAST_PTR(SmPlane, pSurfaceB);
//  if (!pPlaneB) { SER(SM_ERR); } // Not a plane
//
//  // extend the domains the two SmPlane objs to be large enough to include each other's 3d corners
//  double dDist;
//
//  // Use the domains of the faces if available.
//  SmFace *pFace = SM_CAST_PTR( SmFace, pPlaneB->GetOwner() );
//  SmExtent2d sDom = ( pFace ) ? pFace->GetUVDomain() : pPlaneB->GetNaturalUVDomain();
//  SER( pPlaneA->CoverCoincidentSurface( pPlaneB, dTol, bSuccess, dDist, &sDom ));
//  if (!bSuccess) 
//    { SER(SM_ERR); }
//
//  pFace = SM_CAST_PTR( SmFace, pPlaneA->GetOwner() );
//  sDom = ( pFace ) ? pFace->GetUVDomain() : pPlaneA->GetNaturalUVDomain();
//  SER( pPlaneB->CoverCoincidentSurface( pPlaneA, dTol, bSuccess, dDist, &sDom ));
//  if (!bSuccess) 
//    { SER(SM_ERR); }

  // intersect, merge, and relate the intersection geometry between the two topology graphs
  SER(sTI.IntersectInsertRelate());

  // increment and lock unlocked mark for both pBrepContext and pOtherContext
  SmNewMarkAndLock sMarkLock ( (SmContext *)pBrepA->GetContext(), 
                               (SmContext *)pBrepB->GetContext()) ;   // increment and lock any unlocked mark for BrepA and BrepB
  SmMarkType       eBrepMarkType  = sMarkLock.GetMarkType1() ;
  SmMarkType       eOtherMarkType = sMarkLock.GetMarkType2() ;
  
  // get coincident edges
  SmTArray<SmEdge*> sEdges, sOtherEdges;
  sTI.GetCommonEdges(sEdges,sOtherEdges);

  // mark every pBrebA coincident intersection edge and its partner in pBrepB
  for (ii=0; ii<sEdges.GetSize(); ii++) 
    {
      SmEdge *pEdge      = sEdges[ii];
      SmEdge *pOtherEdge = sOtherEdges[ii];

      pEdge->Mark(eBrepMarkType);
      pOtherEdge->Mark(eOtherMarkType);

#ifdef SM_DEBUG_CODE
      if (bDebugMe) 
        {
          if (ii==0) { smgfx_Erase(); }
          smgfx_SetLook(2,2, 0,0,0) ; if(pEdge) pEdge->DrawParams(); sm_GraphicsLoop();
          smgfx_SetLook(2,2, 0,1,0) ; if(pOtherEdge) pOtherEdge->DrawParams(); sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE
    } // end iter every coincident intersection edge
  
  // get coincident vertices
  SmTArray<SmVertex*> sVertices, sOtherVertices;
  sTI.GetCommonVertices(sVertices,sOtherVertices);
  
  // mark every pBrepA coincident intersection vertex and its pBrepB partner
  for (ii=0; ii<sVertices.GetSize(); ii++) 
    {
      SmVertex *pVertex      = sVertices[ii];
      SmVertex *pOtherVertex = sOtherVertices[ii];

      pVertex->Mark(eBrepMarkType);
      pOtherVertex->Mark(eOtherMarkType);
      
#ifdef SM_DEBUG_CODE
      if (bDebugMe) 
        {
          smgfx_SetLook( 2,6, 0,0,0 ); pVertex->Draw(); sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE
    } // end iter every coincident intersection vertex

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe10 = FALSE;
        if (bDebugMe10) 
          {
            sTI.Dump();
            smgfx_Erase();
            smgfx_SetLook( 1,2, 0,0,1 ); pBrepA->Draw(1); sm_GraphicsLoop();
            smgfx_SetLook( 3,5, 1,0,1 ); sTI.Draw(3,5, 7,9);   sm_GraphicsLoop();
            sm_GraphicsLoop();
          }
#endif // SM_DEBUG_CODE
  
  SmTArray<SmEdgeuse*> sEdgeuses;

  // mark all coincident faces found connected to coincident edges

  // for every intersection edge (coincident edges),
  // call FindRadialSector() to check for coincident faces.
  for (ii=0; ii<sEdges.GetSize(); ii++) 
    {
      SmEdge *pEdge      = sEdges[ii];
      SmEdge *pOtherEdge = sOtherEdges[ii];
      
      // iter every other pEdge->edgeuse (skip edgeuse mates)
      pEdge->GetEdgeuses(sEdgeuses);
      for (jj=0; jj<sEdgeuses.GetSize(); jj+=2) 
        {
          SmEdgeuse * pEU   = sEdgeuses[jj];
          SmFace    * pF = pEU->GetFace();

          // If the face is marked we already classified it
          if (pF->IsMarked(eBrepMarkType)) { continue; }
          pF->Mark(eBrepMarkType);
          
          // If we make it to here this face has not yet been classified
          // Classify it now relative to the other planar region
          SmEdgeuse *pFoundEU = NULL;
          SmFaceuse *pFoundFU = NULL;
          SER(pOtherEdge->FindRadialSector(pEU,NULL,SM_OT_SAME,pFoundEU,pFoundFU));
          
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe5 = FALSE;
          if(bDebugMe5) 
            {
              smgfx_Erase();
              smgfx_SetLook( 2,2, 1,0,0); pEU  ->Draw(); sm_GraphicsLoop();
              smgfx_SetLook( 2,2, 0,0,1); pF->Draw(); sm_GraphicsLoop();
              if (pFoundFU) { pFoundFU->GetFace()->Draw(); }
              sm_GraphicsLoop();
            }
#endif // SM_DEBUG_CODE
          // when a radial sector was found - relate and mark the faces
          if (pFoundFU) 
            {
              SmFace *pOFace = pFoundFU->GetFace();
              pOFace->Mark(eOtherMarkType);
              sTI.Relate(pF,pOFace);
            }
        } // For edge edgeuse in edge
      
      // iter every other pOtherEdge->edgeuse (skip edgeuse mates)
      pOtherEdge->GetEdgeuses(sEdgeuses);
      for (jj=0; jj<sEdgeuses.GetSize(); jj+=2) 
        {
          SmEdgeuse *pEU    = sEdgeuses[jj];
          SmFace    *pOFace = pEU->GetFace();

          // If the face is marked we already classified it
          if (pOFace->IsMarked(eOtherMarkType)) { continue; }
          pOFace->Mark(eOtherMarkType);

          // If we make it to here this face has not yet been classified
          // Classify it now relative to the brep solid
          SmEdgeuse *pFoundEU = NULL;
          SmFaceuse *pFoundFU = NULL;
          if (pEdge->FindRadialSector(pEU,NULL,SM_OT_SAME,pFoundEU,pFoundFU) != SM_SUCCESS) 
            {
              continue;  // If we fail just go to different edgeuse
            }
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe6 = FALSE;
          if (bDebugMe6) 
            {
              smgfx_Erase();
              smgfx_SetLook( 2,2, 1,0,0 ); pEU->Draw(); sm_GraphicsLoop();
              smgfx_SetLook( 2,2, 0,1,0 ); pOFace->Draw(SM_DM_CROSSHATCH,10,10); sm_GraphicsLoop();
              smgfx_SetLook( 2,2, 0,0,1 ); if (pFoundFU) pFoundFU->GetFace()->Draw(SM_DM_CROSSHATCH,10,10); sm_GraphicsLoop();
              sm_GraphicsLoop();
            }
#endif // SM_DEBUG_CODE
          
          // First handle the coincident face case
          if (pFoundFU) 
            {
              SmFace *pF = pFoundFU->GetFace();
              pF->Mark(eBrepMarkType);
              SER(sTI.Relate(pF,pOFace));
            }
        } // end iter every OtherEdge->edgeuse 
      
    } // end iter every coincident edge   

#ifdef SM_DEBUG_CODE
  if (bDebugMe10) 
    {
      sTI.Dump();
      smgfx_Erase();
      smgfx_SetLook( 1,2, 0,0,1 ); pBrepA->Draw(1); sm_GraphicsLoop();
      smgfx_SetLook( 3,5, 1,0,1 ); sTI.Draw(3,5, 7,9);   sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE
  
  // Now we should have all common faces marked - 
  // remove all pBrepA Faces no longer needed
  SmTArray<SmFace*> sDelFaces;

  // for every pBrepA face - decide which need to be deleted
  pBrepA->GetFaces(sFaces);
  for (ii=0; ii<sFaces.GetSize(); ii++) 
    {
      SmFace    *pF        = sFaces[ii];
      SmFace    *pOFace       = SM_CAST_PTR(SmFace,sTI.GetOtherMate(pF));
      SmBoolean  bCommonFace  =   (pOFace) 
                                ? TRUE
                                : FALSE;

      // switch on eBooleanType to decide which faces to delete
      switch (eBooleanType) 
        {
          case SM_2D_EXCLUSIVE_OR:
          case SM_2D_DIFFERENCE  : if ( bCommonFace) sDelFaces.Add(pF) ;
                                   break ;

          case SM_2D_INTERSECTION: if (!bCommonFace) sDelFaces.Add(pF) ;
                                   break ;

          default:                 break ;

        } // end switch on eBooleanType

    } // end for each BrepA Face deciding which to delete

  // But don't actually remove them yet, that would mess up deciding
  // which Faces to copy from pBrepB back into pBrepA.  [B467]

  // copy required pBrepB faces into pBrepA
    {
      // CreateCurvesFromFace() locals
      SmTArray<ULONG>           sCurveLoops;
      SmTArray<SmCurve*>        s3DCurves;
      SmTArray<SmBSplineCurve*> sUVCurves;
      SmTArray<SmOrientType>    sCurveOrients;
      SmTArray<SmPoint3d>       sLoopPoints;
    
      // for every pBrepB face - decide which faces to copy into pBrepA
      pBrepB->GetFaces(sFaces);
      for (ii=0; ii<sFaces.GetSize(); ii++) 
        {
          SmFace    *pOFace      = sFaces[ii];
          SmFace    *pF          = SM_CAST_PTR(SmFace,sTI.GetBrepMate(pOFace));
          SmBoolean  bCommonFace =   (pF) 
                                   ? TRUE
                                   : FALSE ;
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe8 = FALSE;
          if (bDebugMe8) 
            {
              smgfx_Erase();
              smgfx_SetLook(1,2, 1,0,0); pOFace->Draw();               sm_GraphicsLoop();
              smgfx_SetLook(1,2, 0,0,1); if (pF) { pF->Draw(); } sm_GraphicsLoop();
              sm_GraphicsLoop();
            }
#endif // SM_DEBUG_CODE

          // switch on eBooleanType to decide which faces to copy into pBrepA 
          switch (eBooleanType)
            {
              case SM_2D_EXCLUSIVE_OR:
              case SM_2D_UNION:
              case SM_2D_MERGE:
                if (!bCommonFace) 
                  {
                    // init CreateCurvesFromFace locals
                    sCurveLoops.ReSet();
                    s3DCurves.ReSet();
                    sCurveOrients.ReSet();
                    sLoopPoints.ReSet();
                    
                    // get copies of the pOFace->edge->curves
                    SER(pOFace->CreateCurvesFromFace(*pContext,SM_OT_SAME,
                                         sCurveLoops,&s3DCurves,NULL,sCurveOrients,sLoopPoints));
                    
                    // MakeFaceWithCurves locals
                    SmRegion *pNewRegion = NULL ;
                    SmShell  *pNewShell  = NULL ;
                    SmFace   *pNewFace   = NULL ;
                    
//cbi_CEdge:
// Remove Composities  
//                    // If the surface already has an owner and we're not making composites, copy the surface.
//                    if ( pPlaneA->GetOwner() != NULL  &&  ! pBrepA->GetMakeComposites() )
                    // If the surface already has an owner, copy the surface.
                    if(pSurface1->GetOwner() != NULL)
                      {
                        SmSurface *pCopySurface = NULL;
                        pSurface1->Copy( *( pSurface1->GetContext() ), pCopySurface );
                        NER( pCopySurface );
                        pSurface1 = pCopySurface;
                      }

                    // create pBrepA face from pBrepB curve copies
                    SER(pBrepA->MakeFaceWithCurves(pBrepA->GetInfiniteRegion(),     // in : region to contain new topology objects
                                                   sCurveLoops,                     // in : 1 entry per loop, value = loop edge count, 1st entry=outer loop
                                                   &s3DCurves,                      // in : opt ordered 3d trimming curves assigned to loops per sLoopEUCounts
                                                   NULL,                            // in : opt ordered 2d trimming curves assigned to loops per sLoopEUCounts
                                                   sCurveOrients,                   // in : associated orients for each trimming curve, SM_OT_SAME or SM_OT_OPPOSITE
                                                   sLoopPoints,                     // in : Point positions to build SmVertex VertexLoops
                                                   pSurface1,                       // in : new face->Surface
                                                   pSurface1->GetNaturalUVDomain(), // in : domain of Surface used by face
                                                   SM_OT_SAME,                      // in : Surface orient, oneof SM_OT_SAME or SM_OT_OPPOSITE
                                                   pNewRegion,                      // out: New region if any. NULL when building trimmed surfaces, may be NotNULL for solids.
                                                   pNewShell,                       // out: New shell if any.  Trimmed surfaces always create a new shell.
                                                   pNewFace));                      // out: the new face
                  }
                break ;

              default: 
                break ;
            } // end switch on eBooleanType
        } // end iter every pBrepB face


      // Now we can remove all pBrepA Faces no longer needed.
        {
          sTI.DeleteFaces(sDelFaces);
        }


#ifdef SM_DEBUG_CODE
      if (bDebugMe10) 
        {
          sTI.Dump();
          smgfx_Erase();
          smgfx_SetLook( 1,2, 0,0,1 ); pBrepA->Draw(1); sm_GraphicsLoop();
          smgfx_SetLook( 3,5, 1,0,1 ); sTI.Draw(3,5, 7,9);   sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE

      // Modified this:
      // 1. It's required for Union even if no faces were added here. [B294]
      // 2. Delete only those manifold edges that were involved in this operation.
      //
      // When doing an SM_2D_UNION operation:
      // a. stitch faces to remove coincident edges
      // b. make manifold
      //
      // Note: we definitely do want to do this for Union, and definitely not
      // for Merge.  For the other three (Intersect, Difference, ExclusiveOr),
      // it seems it would never arise.  We can check those cases anyway.
      if ( eBooleanType != SM_2D_MERGE )
        {
          ULONG lStitchedEdges, lLaminaEdges;
          double dMaxVGap = 0.0, dMaxEGap = 0.0;
      
          // stitch pBrepA faces - glue all vertex and edge pairs coincident to within pBrepA->GetTolerance()
          SER(pBrepA->StitchFaces(pBrepA->GetTolerance(),
                               lStitchedEdges,lLaminaEdges,
                               dMaxVGap,
                               dMaxEGap));
      
          // Remove the internal topological edges and vertices while preserving
          // a surface whose parameter domain covers both merged faces.  Calling
          // DeleteEdge() directly can leave the surviving face with one input
          // fragment's UV domain when the coincident planes are distinct
          // surface objects.
          SER(pBrepA->RemoveTopologicalEdgesAndVertices());
        } // end need to stitch and makeManifold check
    } // end scope: copying BrepB faces into BrepA faces

  // set result = pBrepA. Delete pBrepB
  rpResult = pBrepA;
  sStack2.Clear();
  SM_ASSERT(pBrepB != NULL) ; delete pBrepB ; pBrepB = NULL ;
  
  // all done
  return SM_SUCCESS;

} // end SmPrimitiveCreation::Boolean2D

/*******************************************************************//**
PURPOSE: 
      
NOTES: 
***********************************************************************/
void SmPrimitiveCreation::InitEntityMaps()
{
  m_vMapToSame.   RemoveAll();
  m_vMapToHigher. RemoveAll();
  m_vMapFromSame. RemoveAll();
  m_vMapFromLower.RemoveAll();
}

/*******************************************************************//**
PURPOSE: 
      
NOTES: 

***********************************************************************/
void SmPrimitiveCreation::CopyEntityMaps( SmTopologySweep *pSweep )
{
  *(pSweep->GetMapToSame()   ) = m_vMapToSame   ;
  *(pSweep->GetMapToHigher() ) = m_vMapToHigher ;
  *(pSweep->GetMapFromSame() ) = m_vMapFromSame ;
  *(pSweep->GetMapFromLower()) = m_vMapFromLower;
}

/*******************************************************************//**
PURPOSE: 
      
NOTES: 
***********************************************************************/
void SmPrimitiveCreation::AppendEntityMaps( SmTopologySweep *pSweep )
{
  pSweep->GetMapToSame()   ->Insert( m_vMapToSame    );
  pSweep->GetMapToHigher() ->Insert( m_vMapToHigher  );
  pSweep->GetMapFromSame() ->Insert( m_vMapFromSame  );
  pSweep->GetMapFromLower()->Insert( m_vMapFromLower );
}
