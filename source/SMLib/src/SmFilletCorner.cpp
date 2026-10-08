// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmFilletCorner.cpp
* PURPOSE: Source code file for SmFilletCorner object.
**********************************************************************/

#include "StdAfx.h"

#ifndef __SMFILLETCORNER_H__
#include <SmFilletCorner.h>
#endif

#ifndef __SMFILLETSOLVER_H__
#include <SmFilletSolver.h>
#endif

#ifndef __SMFILLETSTANDARDSOLVER_H__
#include <SmFilletStandardSolver.h>
#endif

#include <SmCompositeCurve.h>
#include <SmPlane.h>
#include <SmLine.h>
#include <SmGeomUtility.h>

#ifdef SM_GFX_CODE
  #include <SmGraphicsOutput.h>
#endif // SM_GFX_CODE

//#define VALIDATE_POINTERS 1


/*******************************************************************//**
    Static functions
***********************************************************************/

/********************************************************************
PURPOSE: Given a side edgeuse determine whether itself or its radial
            is the one which FilletSolver use in solving Rail_x_SideEdge
NOTES:
********************************************************************/
static SmEdgeuse * sm_FindSideEdgeuse      // rtn: edgeuse
 (SmEdgeuse * pFilEdgeuse,           // in : target fillet edgeuse
  SmEdgeuse * pSideEU,               // in : target sideEdgeuse - connects to a filleted edge at an filletCorner
  int         iDebugLevel)
{
  // locals - filleted edge and CW neighbor to SideEU
  SmEdge    * pFilletedEdge = pFilEdgeuse->GetEdge();
  SmEdgeuse * pEU           = pSideEU;
  SmEdge    * pE            = pEU->GetCWEdgeuse()->GetEdge();

  // when edgeuses connect to different edges
  if (pFilletedEdge != pE)
    {
      // check the radial edgeuse
      pEU = pEU->GetRadial();
      pE  = pEU->GetCCWEdgeuse()->GetEdge();
      if (pFilletedEdge != pE)
          return NULL;
    }

#ifdef SM_DEBUG_CODE
  if ( iDebugLevel ) 
    {
      smgfx_SetLook(1,2, 1,0,0); pFilEdgeuse->Draw() ; sm_GraphicsLoop();
      smgfx_SetLook(1,2, 1,0,0); pEU->Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#else
    SM_REF1(iDebugLevel);
#endif // SM_DEBUG_CODE

  return pEU;

} // end sm_FindSideEdgeuse

/********************************************************************
PURPOSE: Given an index of rail and filleted edgeuse determine whether
            a given closed-side-edgeuse or its mate is the one which
            FilletSolver use in solving Rail_x_SideEdge.
            In a 2x1-on-non-closed-edge case, the side edge should always be a
            closed but non-periodic. Therefore, we need to carefully choose
            which side edge correspond to which rail. Here, our decision is to
            choose side edgeuse on the rail-0 side to be the edgeuse that goes
            'from' the end vertex of the filleted edge and choose the one that goes
            'into' the end vertex of the filleted edge for rail1

NOTES:
********************************************************************/
static SmEdgeuse * sm_FindClosedSideEdgeuse
 (ULONG            lRailIndex,
  SmFilletSolver * pFilSolver,
  SmEdgeuse      * pSideEU,
  int              iDebugLevel)
{
  SmEdgeuse * pFilEdgeuse = pFilSolver->GetEdgeuse(0);
  SmVertex  * pV          = pFilEdgeuse->GetVertexuse()->GetVertex();
  SmVertex  * pSideV      = pSideEU->GetVertexuse()->GetVertex();
  SmEdgeuse * pEU         = (lRailIndex == 0) ? ( (pSideV == pV) ? pFilEdgeuse->GetCWEdgeuse()->GetMate()
                                                                 : pFilEdgeuse->GetCCWEdgeuse() )
                                              : ( (pSideV == pV) ? pFilEdgeuse->GetCWEdgeuse()
                                                                 : pFilEdgeuse->GetCCWEdgeuse()->GetMate() ) ;
#ifdef SM_DEBUG_CODE
  if ( iDebugLevel > 0 ) 
    {
      smgfx_SetLook(1,2, 1,0,0); pFilEdgeuse->Draw(); sm_GraphicsLoop();
      pEU->Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#else
  SM_REF1(iDebugLevel);
#endif // SM_DEBUG_CODE

  // all done
  return pEU;

} // end sm_FindClosedSideEdgeuse

/********************************************************************
PURPOSE:  Determine if two adjacent edges are tangent to each other
             at a given vertex
NOTES:
********************************************************************/
static SmBoolean sm_TestG1Continuity // rtn: TRUE = Edge EndTangents at vertex are within tolerance of being tangent
  (const SmVertex * pVert,           // in : Target Vertex
   const SmEdge   * pEdge1,          // in : 1st edge connecting to vertex
   const SmEdge   * pEdge2,          // in : 2nd edge connectint to vertex
   double           dAngTolDeg,      // in : Angle tolerance in degrees
   double         * pdAngDeg,        // out: angle (degrees) between edges at vertex
   int              iDebugLevel)     // in : 
{
  // no work - not given 3 valid pointers
  if (   pVert  == NULL
      || pEdge1 == NULL
      || pEdge2 == NULL) 
   { return FALSE ; }

  // locals
  SmVector3d  sPV1[2] ;
  SmCurve   * pCrv1   = pEdge1->GetCurve() ;
  SmExtent1d  sIvl1   = pEdge1->GetInterval() ;
  double      dParam1 = (pEdge1->GetStartVertex() == pVert) ? sIvl1.GetMin() : sIvl1.GetMax() ;

  // get Edge1 tangent at vertex
  pCrv1->Evaluate(dParam1,1,TRUE,sPV1) ;

  // locals
  SmVector3d  sPV2[2];
  SmCurve   * pCrv2   = pEdge2->GetCurve();
  SmExtent1d  sIvl2   = pEdge2->GetInterval();
  double      dParam2 = (pEdge2 == pEdge1 || pEdge2->GetStartVertex() != pVert) ? sIvl2.GetMax() : sIvl2.GetMin() ;

  // get Edge2 tangent at vertex
  pCrv2->Evaluate(dParam2,1,TRUE,sPV2);
  
#ifdef SM_DEBUG_CODE
    if ( iDebugLevel > 0 ) 
      {
        smgfx_SetLook(1,6, 1,0,0); pVert->Draw(); sm_GraphicsLoop();
        sPV1[1].Draw(sPV1); sm_GraphicsLoop();
        sPV2[1].Draw(sPV2); sm_GraphicsLoop();
        sm_GraphicsLoop();
      }
#else
  SM_REF1(iDebugLevel);
#endif // SM_DEBUG_CODE

  if (   sPV1[1].LengthSquared() > SM_EFF_ZERO_SQ
      && sPV2[1].LengthSquared() > SM_EFF_ZERO_SQ)
    {
      // get angle (degrees) between Edge1 and Edge2 endTangents
      double dAngDeg;
      SE(sPV1[1].AngleBetween(sPV2[1],dAngDeg));
      if (dAngDeg > SM_PI/2.0) dAngDeg = SM_PI - dAngDeg;
      dAngDeg = SM_RAD2DEG(dAngDeg);

      // set optional output
      if (pdAngDeg) { *pdAngDeg = dAngDeg; }

      // check for in tolerance angle
      if (dAngDeg < dAngTolDeg)
        {
          return TRUE;
        }
    } // end nonDegenerate endTangent check

  // endTangents are not within tolerance
  return FALSE;

} // end sm_TestG1Continuity

/********************************************************************
PURPOSE:  return face connected to sectors of two given edgeuses,
             return NULL when no such face exists.

NOTES:
   use pEU3 to specify a face to skip, this forces the function
   to return another face that connects both edges or NULL.

   When pEU3 != NULL, the function returns NULL when the
     edges are not connected to a common face or when
     they are only connected to one common face which
     is equal to pEU3->Face.

METHOD ---
  4 faces are examined pEU1->SectorBeginFace, pEU1->SectorEndFace
                       pEU2->SectorBeginFace, pEU2->SectorEndFace
  and the common face is returned.
  Note in nonManifold modeling not all faces attaced to pEU->edges are examined.

********************************************************************/
static SmFace * sm_GetFaceByTwoEdgeuses
 (SmEdgeuse * pEU1,       // in : bounding edge 1
  SmEdgeuse * pEU2,       // in : bounding edge 2
  SmEdgeuse * pEU3)       // in : opt edgeuse to specify a face to ignore.
                          //      function will not return pEU3->Face.
                          //      NULL to ignore.
{
  if (   pEU1 == NULL
      || pEU2 == NULL) 
    { return NULL; }

  // locals
  SmTArray<SmFace*> sFaces1;
  SmTArray<SmFace*> sFaces2;

  // let sFaces1 = two faces attached to pEU1->Sector
  sFaces1.Add(pEU1->GetFace());
  sFaces1.Add(pEU1->GetRadial()->GetFace());

  // let sFaces2 = two faces attached to pEU2->Sector
  sFaces2.Add(pEU2->GetFace());
  sFaces2.Add(pEU2->GetRadial()->GetFace());

  // for every pEU1->Sector->Face
  ULONG ii, jj;
  for (ii=0; ii<sFaces1.GetSize(); ii++)
    {
      SmFace * pFace1 = sFaces1[ii];

      // when asked skip pEU3->Face
      if (pEU3 && pEU3->GetFace() == pFace1) 
        { continue; }

      // for every pEU2->Sector->Face
      for (jj=0; jj<sFaces2.GetSize(); jj++)
        {
          SmFace * pFace2 = sFaces2[jj];

          // return 1st found common face
          if (pFace1 == pFace2)
            {
              return pFace2;
            }
        } // end iter every pEU2->Sector->Face
    } // end iter every pEU1->Sector->Face

  return NULL;

} // end sm_GetFaceByTwoEdgeuses

/********************************************************************
PURPOSE: Determine if the edgeuse bounded two tangential faces
            with a specified angle tolerance
NOTES:
********************************************************************/
static SmBoolean sm_CheckTangentialEdgeuse
 (SmEdgeuse * pEU,
  double      dAngTolDeg)
{
  SmVector3d sPnt1, sPnt2, sBin1, sBin2, sNorm1, sNorm2;

  double dT = pEU->GetEdge()->GetInterval().Evaluate(0.5);

  SE(pEU->EvaluateBinormal(dT,FALSE,sPnt1,sBin1,NULL,&sNorm1));
  SE(pEU->GetRadial()->EvaluateBinormal(dT,FALSE,sPnt2,sBin2,NULL,&sNorm2));

  if (sNorm1.IsParallelTo(sNorm2,dAngTolDeg)) 
    {
      // Edge is adjacent to two tangential faces
      return TRUE;
    }

  return FALSE;

} // end sm_CheckTangentialEdgeuse

/********************************************************************
PURPOSE: Classify a face's corner at a given vertex as convex or
    concave by examining the shape of the face->Corners bounding edgeuses.
    For a 3x2 vertex (pOptRefEU NULL), this checks for mixed convexity:
    the third edges will have opposite convexity to the two given edges.

    return FALSE When inside face angle runs from 0 to 180+tol   degrees
           TRUE  when inside face angle runs from 180+tol to 360 degrees

        |\\ - face    \\\\\|  CONCAVE        Special Case: tangential edgeuses
 CONVEX |\\\   inside -\\\\|   FACE CORNER     Classified as CONVEX CORNER
        +-----          \\\+-----               -----+-----
    FACE CORNER          \\\\\\\\                \\\\\\\\\\
                           \\\\\\\                \\\\\\\\\

NOTES:
    The optional reference edgeuse, if given, should
       'lie' between two test edgeuses. i.e. two test edgeuses
       are no longer bounding the same face.

********************************************************************/
static SmBoolean sm_CheckFaceComplimentaryCornerConcavity (
 const SmVertex * pCornerVert, // in : target corner vertex
 SmEdgeuse      * pTestEU1,    // in : 1st target edgeuse - points outward from CornerVert
 SmEdgeuse      * pTestEU2,    // in : 2nd target edgeuse - points outward from CornerVert
 SmEdgeuse      * pOptRefEU,   // in : NULL    = EU1 and EU2 are immediate neighbors bounding same face
                               //      notNULL = OptRefEU lies 'between' EU1 and EU2 so that EU1->edge and
                               //        EU2->edge do not bound the same face. Needs to be a tangential edge.
 int              iDebugLevel) // in :
{
  // locals
  SmEdgeuse * pEU1 = pTestEU1;
  SmEdgeuse * pEU2 = pTestEU2;

  // Note: Here we test against 2 degrees.
  // In some other places, they test against 5 degrees.
  double dAngleLimitDegrees = 2.0;

  // Do NOT alter the following algorithm unless
  // you understand radial-edge topology

  // If pOptRefEU is given, it needs to be a tangential edge
  // Typically, this is happenning in a 4x2 concave corner
  if(   pOptRefEU
     && sm_CheckTangentialEdgeuse(pOptRefEU, dAngleLimitDegrees ) == FALSE)
    { return FALSE ;}

  // get face spanning the edgeuse->Edges or any old face connected to the pOptRefEU->edge
  SmFace * pCommonFace =   (pOptRefEU)
                         ? pOptRefEU->GetFace()
                         : sm_GetFaceByTwoEdgeuses(pTestEU1,pTestEU2,NULL) ;

  // gwc:change - made check work for both edges
  // let pEU2 and pEU1 be Edgeuses to common face from their respective edges
  if (pCommonFace == pTestEU2->GetRadial()->GetFace())
    {
      pEU2 = pTestEU2->GetRadial();
    }
  if (pCommonFace == pTestEU1->GetRadial()->GetFace())
    {
      pEU1 = pTestEU1->GetRadial();
    }

  // arrive here when pEU1 and pEU2 point into common face

  // get parameter on pEU1 corresponding to CornerVert
  SmVertex * pVert1 = pEU1->GetVertexuse()->GetVertex();
  double     dT1    =  (   (pCornerVert == pVert1 && pEU1->GetOrientation() == SM_OT_OPPOSITE)
                        || (pCornerVert != pVert1 && pEU1->GetOrientation() == SM_OT_SAME))
                      ? 1.0
                      : 0.0;

  // get pEU1 tangent (pointing from corner into edge) and binormal at cornerVertex
  SmPoint3d  sBinPnt1;
  SmVector3d sBinVec1, sTangent1;
  SmExtent1d sIvl1 = pEU1->GetEdge()->GetInterval();
  SE(pEU1->EvaluateBinormal(sIvl1.Evaluate(dT1),FALSE,sBinPnt1,sBinVec1,&sTangent1));

  if ( pCornerVert != pVert1 )  // our corner vertex is the end of the edge,
    {
      sTangent1 = -sTangent1;   // so flip the tangent to point away from vtx
    }

  // get parameter on pEU2 corresponding to CornerVert
  SmVertex * pVert2 = pEU2->GetVertexuse()->GetVertex();
  double     dT2    =  (   (pCornerVert == pVert2 && pEU2->GetOrientation() == SM_OT_OPPOSITE)
                        || (pCornerVert != pVert2 && pEU2->GetOrientation() == SM_OT_SAME))
                      ? 1.0
                      : 0.0;

  // get pEU2 tangent (pointing from corner into edge) and binormal at cornerVertex
  SmPoint3d  sBinPnt2;
  SmVector3d sBinVec2, sTangent2;
  SmExtent1d sIvl2 = pEU2->GetEdge()->GetInterval();
  SE(pEU2->EvaluateBinormal(sIvl2.Evaluate(dT2),FALSE,sBinPnt2,sBinVec2,&sTangent2));
  if (pCornerVert != pVert2)
    {
      sTangent2 = -sTangent2;
    }

#ifdef SM_DEBUG_CODE
  // draw corner vertex(red), edgeuses(green,yellow), commonFace(black), and tangents/binormals(cyan/Blue)
  if ( iDebugLevel > 0 )
    {
     // double dValT1 = sBinVec1.Dot(sTangent1);
     // double dValT2 = sBinVec2.Dot(sTangent2);

      // draw vert, edgeuses, and face
      smgfx_SetLook(3,5, 1,0,0); pCornerVert->Draw() ;     sm_GraphicsLoop();
      smgfx_SetLook(3,5, 0,1,0); pEU1->Draw();             sm_GraphicsLoop();
      smgfx_SetLook(3,5, 1,1,0); pEU2->Draw();             sm_GraphicsLoop();
      smgfx_SetLook(3,5, 0,0,0); if(pCommonFace) pCommonFace->DrawUV(5,5); sm_GraphicsLoop();

      // draw tangents
      SmPoint3d sPoint = pCornerVert->GetPoint();
      smgfx_SetLook(4,6, 0,1,1) ; sTangent1.Draw(&sPoint) ; sm_GraphicsLoop();
      smgfx_SetLook(4,6, 0,1,1) ; sBinVec1. Draw(&sBinPnt1) ;                sm_GraphicsLoop();
      smgfx_SetLook(4,6, 0,0,1) ; sTangent2.Draw(&sPoint) ; sm_GraphicsLoop();
      smgfx_SetLook(4,6, 0,0,1) ; sBinVec2. Draw(&sBinPnt2) ;                sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#else
    SM_REF1(iDebugLevel);
#endif // SM_DEBUG_CODE

  // Check if binormal of each edgeuse is pointing opposite way of
  // the tangent of the other edgeuse
  sBinVec1.Unitize();
  sBinVec2.Unitize();
  sTangent1.Unitize();
  sTangent2.Unitize();
  double dVal1 = sBinVec1.Dot(sTangent2);
  double dVal2 = sBinVec2.Dot(sTangent1);

  double dAngleLimitRadians = SM_DEG2RAD( -dAngleLimitDegrees );
  if (   dVal1 < dAngleLimitRadians
      && dVal2 < dAngleLimitRadians )
    {
      return TRUE;
    }

  return FALSE;

} // end sm_CheckFaceComplimentaryCornerConcavity

/*******************************************************************//**
PURPOSE: Determine whether two edges
            have the same or opposite convexity.

NOTES:
  The edges are given by an edgeuse of each.

***********************************************************************/
static SmBoolean same_convexity_edges // rtn: TRUE=convexities are the same, FALSE=convexities are opposite
 (SmEdgeuse      * pEdgeuse1,         // in : edgeuse of first edge
  SmEdgeuse      * pEdgeuse2,         // in : edgeuse of second edge
  const SmVertex * pVertex,           // in : common vertex between the two edges
  int              iDebugLevel)       // in :
{
  // Method: First, find the edgeuse of the first edge that is in the
  // concave sector (call it pConcaveEU1), then find the edgeuse of
  // the edge indicated by pEdgeuse2 that's in the same region as
  // pConcaveEU1, and see whether it's concave.

  // Get the edgeuse that's in the concave sector of pEdgeuse1's Edge
  // (It's either pEdgeuse1, or its mate's radial.)
  SmEdgeuse * pConcaveEU1 =   pEdgeuse1->IsConvexRadialSector(5)
                            ? (    pEdgeuse1->GetEdge()->IsManifold()
                                 ? pEdgeuse1->GetMate()->GetRadial()
                                 : NULL
                              )
                            : pEdgeuse1 ;
  SM_ASSERT( pConcaveEU1 && ! pConcaveEU1->IsConvexRadialSector(5) )
  if ( pConcaveEU1 == NULL ) { SER(SM_ERR) ; }

  // Next, get Edge2->Edgeuse that's in the same region as pConcaveEU1
  SmEdge * pEdge2 = pEdgeuse2->GetEdge() ;
  pEdgeuse2       = pConcaveEU1->GetCornerMateEdgeuse( pVertex );
  if ( pEdgeuse2->GetEdge() != pEdge2 )
    {
      pEdgeuse2 = pConcaveEU1->GetRadial()->GetCornerMateEdgeuse( pVertex );
    }
  SM_ASSERT( pEdgeuse2->GetEdge() == pEdge2 );

#ifdef SM_DEBUG_CODE
  // draw pEdgeuse1(red), pConcaveEU1(green), final pEdgeuse2(blue)
  if( iDebugLevel )
    {
      SmRegion * pConcaveRegion  = pConcaveEU1->GetShell()->GetRegion() ;
      //SmRegion * pInfiniteRegion = pConcaveEU1->GetShell()->GetBrep()->GetInfiniteRegion() ;
      SmRegion * pSecondRegion   = pEdgeuse2->GetShell()->GetRegion() ;
      SM_ASSERT(pConcaveRegion == pSecondRegion) ;

      if ( FALSE ) 
        {
          SmBrep *pBrep = pConcaveEU1->GetShell()->GetBrep();
          if ( pBrep ) 
            {
              smgfx_Erase();
              smgfx_SetLook( 1,2, 0,0,0 ); pBrep->Draw(TRUE); sm_GraphicsLoop();
            }
        }

      smgfx_SetLook(4,6, 1,0,0) ; pEdgeuse1->Draw() ;   sm_GraphicsLoop() ;
      smgfx_SetLook(4,6, 0,1,0) ; pConcaveEU1->Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(4,6, 0,0,1) ; pEdgeuse2->Draw() ;   sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#else
  SM_REF1(iDebugLevel);
#endif // SM_DEBUG_CODE

  // Got the edgeuse of Edge2 in the same region as the concave sector
  // of the first edge -- return FALSE if it's convex.
  return( pEdgeuse2->IsConvexRadialSector(5) ? FALSE : TRUE );

} // end same_convexity_edges

/********************************************************************
PURPOSE: Find all edgeuses (usually two) of this corner which bound
            a concave face
NOTES:
********************************************************************/
static void sm_FindConcaveEdgeuses
 (const SmVertex       * cpVertex,
  SmTArray<SmEdgeuse*> * pAllEUs,
  SmTArray<SmEdgeuse*> & rConcaveEUs,
  int                    iDebugLevel )
{
  ULONG ii;
  ULONG       lTotalEUs = pAllEUs->GetSize();
  SmEdgeuse * pPrevEU   = (*pAllEUs)[lTotalEUs-1];

  for (ii=0; ii<lTotalEUs; ii++) 
    {
      SmEdgeuse * pCurrEU = (*pAllEUs)[ii];

      if(TRUE == sm_CheckFaceComplimentaryCornerConcavity(cpVertex,
                                                          pPrevEU,
                                                          pCurrEU,
                                                          NULL, 
                                                          iDebugLevel )) 
        {
          rConcaveEUs.Add(pPrevEU);
          rConcaveEUs.Add(pCurrEU);
          return;
        }

      pPrevEU = pCurrEU;
    }

} // end sm_FindConcaveEdgeuses

/********************************************************************
PURPOSE: Compute the angle between two given edgeuses

NOTES:
********************************************************************/
static double sm_FindAngleBetweenEdgeuses
 (const SmVertex * cpCornerVert,
  SmEdgeuse      * pEU1,
  SmEdgeuse      * pEU2,
  int              iDebugLevel )
{
#ifdef SM_DEBUG_CODE
  if ( iDebugLevel ) 
    {
      smgfx_SetLook(1,2, 1,0,0); pEU1->Draw(); sm_GraphicsLoop();
      pEU2->Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#else
    SM_REF1(iDebugLevel);
#endif // SM_DEBUG_CODE

  SmBoolean bAtEdgeStart1 = TRUE;
  SmVertex * pVert1 = pEU1->GetVertexuse()->GetVertex();
  if(   (cpCornerVert == pVert1 && pEU1->GetOrientation() == SM_OT_OPPOSITE)
     || (cpCornerVert != pVert1 && pEU1->GetOrientation() == SM_OT_SAME)) 
    {
      bAtEdgeStart1 = FALSE;
    }

  SmVector3d sTangent1;
  SmEdge   * pE1 = pEU1->GetEdge();

  SER(pE1->GetEndTangent(bAtEdgeStart1,sTangent1));
  if (!bAtEdgeStart1) 
    {
      sTangent1 = -sTangent1;
    }

  SmBoolean  bAtEdgeStart2 = TRUE;
  SmVertex * pVert2        = pEU2->GetVertexuse()->GetVertex();

  if (   (cpCornerVert == pVert2 && pEU2->GetOrientation() == SM_OT_OPPOSITE)
      || (cpCornerVert != pVert2 && pEU2->GetOrientation() == SM_OT_SAME)) 
    {
      bAtEdgeStart2 = FALSE;
    }

  SmVector3d sTangent2;
  SmEdge * pE2 = pEU2->GetEdge();
  SER(pE2->GetEndTangent(bAtEdgeStart2,sTangent2));
  if (!bAtEdgeStart2) 
    {
      sTangent2 = -sTangent2;
    }

  double dAngle;
  SE(sTangent1.AngleBetween(sTangent2,dAngle));

  return dAngle;

} // end sm_FindAngleBetweenEdgeuses

/********************************************************************
PURPOSE: Estimate an extension distance for fillet at Nx1
            or 3x2Mixed corner equal to 10*maxRadius distance

NOTES:
********************************************************************/
static double sm_FindSurfaceExtensionDistAtCorner
 (SmEdgeuse      * pFilEdgeuse,
  SmFilletSolver * pFilSolver,
  const SmVertex * cpNx1CornerVert)
{
  double dExtDist = 0.0;

  switch (pFilSolver->GetSolverType())
    {
      case SM_FS_CONST_RADIUS:
      case SM_FS_CONST_RADIUS_ASSISTED: // return largest fillet radius
        {
          SmConstantRadiusFS * pConstRadiusFS = (SmConstantRadiusFS*)pFilSolver;
          for (ULONG i=0; i<2; i++)
            {
              double dRad = pConstRadiusFS->GetFilletRadius(i);
              if (dRad > dExtDist) dExtDist = dRad;
            }
        }
        break;

      case SM_FS_CONST_DIST: // return constant distance
        {
          SmConstantDistanceFS * pConstDistFS = (SmConstantDistanceFS*)pFilSolver;
          dExtDist = pConstDistFS->GetDistance();
        }
        break;

      case SM_FS_VARIABLE_RADIUS: // return radius at vertex
        {
          SmVariableRadiusFS * pVariableRadiusFS = (SmVariableRadiusFS*)pFilSolver;
          SmExtent1d           sIvl              = pFilEdgeuse->GetEdge()->GetInterval();
          SmVertex           * pVert             = pFilEdgeuse->GetVertexuse()->GetVertex();
          double               dParameter        =  (   (cpNx1CornerVert == pVert && pFilEdgeuse->GetOrientation() == SM_OT_OPPOSITE)
                                                     || (cpNx1CornerVert != pVert && pFilEdgeuse->GetOrientation() == SM_OT_SAME))
                                                   ? sIvl.GetMax()
                                                   : sIvl.GetMin();

          dExtDist = pVariableRadiusFS->GetFilletRadius(dParameter);
        }
        break;

      default:
          SE(SM_ERR);
          break;

    } // end pFilSolver->GetSolverType()

  // In prog_test, Fillet test my_test_suite_0 iter: 2 will create a surface
  // with a slight crease for extension values greater than about 7.5.
  // But, if larger extensions should be required to create a successful fillet,
  // that slight crease is a small price to pay: feel free to increase if needed.
  double dRadFactor = 7.5;
  return dRadFactor * dExtDist;

} // end sm_FindSurfaceExtensionDistAtCorner

/********************************************************************
PURPOSE: This routine will determine whether we can create a surface-of-revolution
            to interpolate the given boundary curves (with end-to-end connections).
            Note: lMajorArcIndex is the index of a curve in the boundary-curves
            array which can be used to determine the axis of revolution
NOTES:
********************************************************************/
static SmStatus sm_CreateSrfOfRevolution
 (const SmContext                 & crContext,      // in : context for new object construction
  const SmTArray<SmBSplineCurve*> & crCurves,       // in : CCW loop of curves to fill with SurfOfRevolution
  ULONG                             lMajorArcIndex, // in : Index of ArcCurve that defines AxisOfRotation and ArcAngle
  double                            d3DTolerance,   // in : MaxAllowed gap between NewSurface and boundary Curves
  // gwc: I think pSideSurface no longer has to be a BBSplineSurface - replace 1 line
  // rm : SmBSplineSurface               *& rpNewSurface,   // out: SurfOfRevolution when BoundaryCurves can be Approximated, else NULL
  SM_FILLETSURF_TYPE             *& rpNewSurface,   // out: SurfOfRevolution when BoundaryCurves can be Approximated, else NULL
  int                               iDebugLevel )   // in : GreaterThan 0 = Add MajorArc Graphics to current display in debug mode
{
  // locals
  ULONG            lTotalCurves = crCurves.GetSize();
  SmBSplineCurve * pMajorArc    = crCurves[lMajorArcIndex];
  
#ifdef SM_DEBUG_CODE
  ULONG di ;
  if ( iDebugLevel > 0 ) // draw input
    {
      SmEdge * pEdge = pMajorArc->GetDim() == 3 ? (SmEdge *)pMajorArc->GetEdge() : NULL ;
      SmBrep * pBrep = pEdge ? pEdge->GetBrep() : NULL ; 

      if(FALSE)
        { smgfx_Erase() ;
          smgfx_SetLook(1,4, 0,0,1); if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
        }
      smgfx_SetLook(2,5, 1,0,1); for(di=0;di<crCurves.GetSize();di++) { if(crCurves[di]) crCurves[di]->DrawParams() ; sm_GraphicsLoop() ;
                                                                        sm_GraphicsLoop() ; 
                                                                      }
      smgfx_SetLook(4,6, 1,0,0); pMajorArc->Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#else
  SM_REF1(iDebugLevel);
#endif // SM_DEBUG_CODE

    SmAxis2Placement sRefFrame;
    double dRad, dStartAng, dEndAng;

  // no work - can't make SurfOfRevolution when MajorArc is not an Arc
  if (!pMajorArc->IsArc(5,            // in : number of points to sample and test
                        d3DTolerance, // in : max deviation from exact Arc allowed each samplePoint
                        sRefFrame,    // out: orientation for found arc,
                                      //      XAxis = centerPoint to StartPoint of curve's interval
                                      //      YAxis = perp to XAxis in direction of Pt on Curve at .15 of interval
                        dRad,         // out: Found Arc radius
                        dStartAng,    // out: Start angle in degrees
                                      //      relative to a counter clockwise angle about Z from 
                                      //      the X axis of reference frame.
                        dEndAng))     // out: End angle in degrees
                                      //      relative to a counter clockwise angle about Z from 
                                      //      the X axis of the reference frame.
    { return SM_ERR ; } // Can not create surf-of-revo
  
  const SmPoint3d & rOrigin    = sRefFrame.GetOriginRef();
  SmVector3d        sRefZAxis  = sRefFrame.GetZAxis();
  double            dAngleSpan = dEndAng - dStartAng;

  // when given exactly 4 curves - make sure OppositeCurve->AxisOfRevolution == MajorCurve->AxisOfRevolution
  //                               and have similar ArcAngle lengths 
  if (lTotalCurves == 4) 
    {
      SmBSplineCurve  * pOtherArc = crCurves[(lMajorArcIndex+2)%4];
      SmAxis2Placement  sRefFrame1;
      double            dRad1, dStartAng1, dEndAng1;

      if (!pOtherArc->IsArc(5,            // in : number of points to sample and test
                            d3DTolerance, // in : max deviation from exact Arc allowed each samplePoint
                            sRefFrame1,   // out: orientation for found arc,
                                          //      XAxis = centerPoint to StartPoint of curve's interval
                                          //      YAxis = perp to XAxis in direction of Pt on Curve at .15 of interval
                            dRad1,        // out: Found Arc radius
                            dStartAng1,   // out: Start angle in degrees
                                          //      relative to a counter clockwise angle about Z from 
                                          //      the X axis of reference frame.
                            dEndAng1))    // out: End angle in degrees
                                          //      relative to a counter clockwise angle about Z from 
                                          //      the X axis of the reference frame.
        { return SM_ERR; }

      const SmPoint3d & rOrigin1    = sRefFrame1.GetOriginRef();
      SmVector3d        sRefZAxis1  = sRefFrame1.GetZAxis();
      double            dAngleSpan1 = dEndAng1 - dStartAng1;

      // Quit: if two Arcs don't share Colinear AxesOfRotation and similar ArcAngle lengths
      if(  !sRefZAxis.IsColinearWith(sRefZAxis1,     // in : Direction to test against lineSeg
                                     rOrigin,        // in : StartPnt of LineSeg:[StartPnt EndPnt]
                                     rOrigin1)       // in : EndPnt   of LineSeg:[StartPnt EndPnt]
         || smos_Fabs(dAngleSpan-dAngleSpan1) > 2.0) 
       { return SM_ERR; }

    } // end need to verify OppositeCurve->AxisOfRevolution == MajorCurve->AxisOfRevolution for 4 curves check

  // arrive here when all tests passed - make a SurfOfRevolution for FilletVertex->FilletSurface

  // Increase angle span by 10% to make sure it's big enough
  dAngleSpan *= 1.1;

  // Let SurfOfRevolution GenCurve = Curve CW from MajorArcCurve
  SmBSplineSurface * pSurface = NULL;
  SmBSplineCurve   * pGenCurve = crCurves[((lMajorArcIndex+lTotalCurves)-1)%lTotalCurves];

  // Sweep GenCurve about MajorArcCurve AxisOfRevolution to make VertexFillet->FilletSurface
  SER(SmBSplineSurface::CreateSurfOfRevolution(crContext,  // in : new object context
                                               pGenCurve,  // in : Generating curve
                                               rOrigin,    // in : Origin of the axis of revoluation
                                               sRefZAxis,  // in : Direction vector of axis of revolution 
                                               dAngleSpan, // in : Angle of revolution (> 0.0 && <= 360.0) in degrees
                                               pSurface)); // out: The new surface 
                                                           // NOTE: Degree 2 circular V direction (constant U iso curves)
  NER(pSurface) ;

  // let OtherCurve = Curve opposite to GenCurve
  SmCurve * pOtherCurve = crCurves[(lMajorArcIndex+1)%lTotalCurves];

  // Check for OtherCurve/SurfOfRevolution coincidence 
  SmSolution sSol;
  SmBoolean  bFoundCoincidence = FALSE;
  SmExtent1d sIvl              = pOtherCurve->GetNaturalInterval();
  SmExtent2d sUVDomain         = pSurface->GetNaturalUVDomain();
  SER(pSurface->SimpleCoincidenceChecker( sUVDomain,         // in : Surface domain of interest
                                         *pOtherCurve,       // in : target curve to check
                                          sIvl,              // in : curve interval of interest
                                          d3DTolerance,      // in : max allowed curve/surface deviation for coincidence
                                          bFoundCoincidence, // out: TRUE = curve is coincident with surface
                                                             //      FALSE= Otherwise
                                          sSol));            // out: Solution: a Range solution.  Contains the
                                                             //      farthest-apart coincident points found.  If
                                                             //      PartialCoincidence is NOT ok, then this will be
                                                             //      the whole curve (if coincident) or nothing (if
                                                             //      not).  But if PartialCoinc is ok, then all we
                                                             //      know for sure is that both Start and End are
                                                             //      coincident, and it's probably coincident
                                                             //      in between.

  // when SurfOfRevolution is not coincident with OtherCurve - don't use it
  if (!bFoundCoincidence) 
    {
      // Not all boundary curves lie on the newly created surface
      SM_ASSERT(pSurface != NULL) ; delete pSurface ; pSurface = NULL ;
      return SM_ERR;
    }

  // set output
  rpNewSurface = pSurface;

  // all done
  return SM_SUCCESS;

} // end sm_CreateSrfOfRevolution

/********************************************************************
PURPOSE:  This routine will determine whether we can create a planar surface
             to interpolate the given boundary curves (with end-to-end connections).
NOTES: Given: 3 pts - make SmPlane surface centered on PtCentroid interpolating the points
              4 pts - make NonRational Bilinear BSplineSurface interpolating given corners
              else  - Signal "Not Yet Implemented" error
********************************************************************/
static SmStatus sm_CreatePlanarSurface
 (const SmContext           & crContext,    // in : context for new object construction
  const SmTArray<SmPoint3d> & rPlanePnts,   // in : ControlPt List ordered:3Pts = any CCW sequence - normal set for right hand rule
                                            //                             4Pts = [Pw[0][0], Pw[1][0], Pw[1][1], Pw[0][1]]
  double                      d3DTolerance, // NotUsed: in : max allowed input point to output plane deviation
                                            // gwc: I think pSideSurface no longer has to be a BBSplineSurface - replace 1 line
                                            // rm : SmBSplineSurface         *& rpNewSurface) // out: for 3 pts - pt interpolating SmPlane centered on PtCentroid,
  SM_FILLETSURF_TYPE       *& rpNewSurface) // out: for 3 pts - pt interpolating SmPlane centered on PtCentroid,
                                            //          4 pts - Bilinear BSplineSurface using pts as corners,
                                            //          else  - signal "Not Yet Implemented" error
{
  SM_REF1(d3DTolerance) ;
  // locals and output 
  ULONG              lTotalPnts = rPlanePnts.GetSize();
  SmBSplineSurface * pPlane     = NULL;

  // when 3 points are given - we can always build an exactly interpolating plane
  if (lTotalPnts == 3) 
    {
      // Make regular plane - centered on Point Centroid and oriented by given points
      SmPoint3d  sOrigin = (rPlanePnts[0]+rPlanePnts[1]+rPlanePnts[2]) / 3.0 ;
      SmVector3d sXAxis  = rPlanePnts[1] - rPlanePnts[0];
      SmVector3d sYAxis  = rPlanePnts[2] - rPlanePnts[0];
      SmVector3d sZAxis  = sXAxis * sYAxis;

      // signal error for degenerate point set
      SER(sZAxis.Unitize());

      // create infinite plane
      pPlane = new (crContext) SmPlane(sOrigin,sZAxis);

      // Adjust the newPlane->UVDomain to fit the given points
      double dSize = sXAxis.Length() + sYAxis.Length();
      SmExtent2d sExt(SmPoint2d(-dSize,-dSize),SmPoint2d(dSize,dSize));
      pPlane->AdjustSTEPUVDomain(sExt);

      // set output
      rpNewSurface = pPlane;
    }
  else if (lTotalPnts == 4)
    {
      // Make NonRational Bilinear BSplineSurface interpolating given corners
      // NOTE: input point order matters.
      SmBSplineSurface *pBSPPlane = NULL ;
      SER(SmBSplineSurface::CreateBilinearSurface(crContext,     // in : context for new object construction
                                                  rPlanePnts[0], // in : ControlPoint[0][0] of 4 corner points to interpolate
                                                  rPlanePnts[1], // in : ControlPoint[1][0] of 4 corner points to interpolate
                                                  rPlanePnts[3], // in : ControlPoint[0][1] of 4 corner points to interpolate
                                                  rPlanePnts[2], // in : ControlPoint[1][1] of 4 corner points to interpolate
                                                  pBSPPlane));   // out: NonRational Bilinear BSplineSurface interpolating given corners
      rpNewSurface = pBSPPlane;
    }
  else // other cases not yet implemented
    {
      // Not yet implemented
      SER(SM_ERR);
    }

  // all done
  return SM_SUCCESS;

} // end sm_CreatePlanarSurface

/********************************************************************
PURPOSE: For use in Variable-radius fillet creation for corner

NOTES:
********************************************************************/
static void sm_GetFilletRadius
 (SmPoint3d  sSurfPnt,
  SmVector3d sSurfNormal,
  SmPoint3d  sSurfPnt2,
  SmVector3d sSurfNormal2,
  double     dOffsetOrientation,
  double   & rdRadius,
  double   & rdAngleSpanRad)
{
  if (dOffsetOrientation > 0.0) 
    {
      sSurfNormal  = -sSurfNormal;
      sSurfNormal2 = -sSurfNormal2;
    }

  SmVector3d sAxis = sSurfNormal*sSurfNormal2;
  sAxis.Unitize();

  double dAngRad;
  sAxis.CCWAngleBetween(sSurfNormal,sSurfNormal2,dAngRad);
  if (dAngRad > SM_PI) 
    {
      dAngRad = SM_PI - dAngRad;
    }

  double dLeng    = sSurfPnt.DistanceBetween(sSurfPnt2);
  rdRadius        = dLeng/2.0/smos_Sine(dAngRad/2.0);
  rdAngleSpanRad  = dAngRad;

} // end sm_GetFilletRadius

/*******************************************************************//**
PURPOSE: If a uv point is outside the domain, check whether it can be moved
   into the domain due to periodicity.  Move it if so, and return TRUE.

NOTES: This is also defined in SmFilletGeom.cpp.
***********************************************************************/
static SmBoolean sm_CheckSeamOut( const SmSurface *pSurf, SmPoint2d & rUV, double dTol, SmSurfParamType eClosure )
{
  if ( eClosure == SM_SP_NEITHER ) { return FALSE; }
  if ( rUV.x   <= -SM_BIG_DOUBLE ) { return FALSE; }
  if ( rUV.y   <= -SM_BIG_DOUBLE ) { return FALSE; }

  SmExtent2d sDom = pSurf->GetNaturalUVDomain();

  if ( sDom.ContainsPoint2d( rUV, dTol ) )
    { return FALSE; }

  SmBoolean bRet = FALSE;

  if ( eClosure == SM_SP_U || eClosure == SM_SP_BOTH )
    {
      SmExtent1d sDomU = sDom.GetUInterval();
      if ( rUV.x < sDomU.GetMin() - dTol )
        {
          rUV.x += sDomU.GetLength();
          bRet = TRUE;
        }
      if ( rUV.x > sDomU.GetMax() + dTol )
        {
          rUV.x -= sDomU.GetLength();
          bRet = TRUE;
        }
    }

  if ( eClosure == SM_SP_V || eClosure == SM_SP_BOTH )
    {
      SmExtent1d sDomV = sDom.GetVInterval();
      if ( rUV.y < sDomV.GetMin() - dTol )
        {
          rUV.y += sDomV.GetLength();
          bRet = TRUE;
        }
      if ( rUV.y > sDomV.GetMax() + dTol )
        {
          rUV.y -= sDomV.GetLength();
          bRet = TRUE;
        }
    }

  return bRet;

} // end static sm_CheckSeamOut

/*******************************************************************//**
PURPOSE: Set surface closure flags.

NOTES: 
***********************************************************************/
static SmSurfParamType sm_SetSurfaceClosure( const SmSurface *pSurf, const SmExtent2d & rDomain )
{
  SmSurfParamType eRet = SM_SP_NEITHER;

  if ( pSurf->IsClosed( rDomain, SM_SP_U ) )
    { eRet = SM_SP_U; }
  if ( pSurf->IsClosed( rDomain, SM_SP_V ) )
    { eRet = ( eRet == SM_SP_NEITHER ) ? SM_SP_V : SM_SP_BOTH; }

  return eRet;
}

/*******************************************************************//**
    END - Static functions
***********************************************************************/


/*******************************************************************//**
PURPOSE: Constructor of default corner

NOTES:
***********************************************************************/
SmFilletCorner::SmFilletCorner
  (const SmContext      & crContext,
   const SmVertex       * cpVertex,
   SmTArray<SmEdgeuse*> * pSolverEUs,
   SmTArray<SmEdgeuse*> * pAllEUs,
   SmFilletExecutive    * pExec,
   double                 dApproxTol3d,
   double                 dTangencyTolRadians)
 : m_crContext(crContext),
   m_pExecutive(pExec),
   m_cpVertex(cpVertex),
   m_pSolverEUs(pSolverEUs),
   m_pAllEUs(pAllEUs),
   m_bBevel(FALSE),
   m_bBlending(FALSE),
   m_bChamfer(FALSE),
   m_bDegenerate(FALSE),
   m_bCalcAsConstRad(FALSE),
   m_bTopologyAdjusted(FALSE),
   m_bExtendedSurfacePatch(TRUE),
   m_dSetBackDist(0.0),
   m_dThisApproxTol3d(dApproxTol3d),
   m_dTangencyTolRadians(dTangencyTolRadians)
{
    m_pPseudoBrep = pExec->m_pPseudoBrep;

} // end SmFilletCorner::SmFilletCorner constructor

//
//    double m_dSetBackDist;          // Distance of setback (from the
//                                    //   filleted vertex).
//                                    // No setbacks if m_dSetBackDist = 0.0
//                                    // Note, this makes no sense with Bevel.
//    double m_dThisApproxTol3d;      // 3D tolerance for corner patch creation
//    double m_dTangencyTolRadians;   // Angle tol for tangent field approximation
//
//    SmFilStatus m_eStatus;          // Status of corner computation

/*******************************************************************//**
PURPOSE: Destructor of default corner

NOTES:
***********************************************************************/
SmFilletCorner::~SmFilletCorner()
{
  for (ULONG i=0; i<m_vVertices.GetSize(); i++) 
    {
      SM_ASSERT(m_vVertices[i] != NULL) ; delete m_vVertices[i] ; m_vVertices[i] = NULL ;
    }
    if (m_pSolverEUs)
    {
        delete m_pSolverEUs;
        m_pSolverEUs = NULL;
    }
  if (m_pAllEUs)    { delete m_pAllEUs;    m_pAllEUs    = NULL ; }

} // end SmFilletCorner::~SmFilletCorner

/*******************************************************************//**
PURPOSE: Register the OriginalFace/ExtSurface pair in the
     SmFilletExecutive::m_vExtendedSurfacesMap list of pairs.

NOTES:
  NOTE: This method might delete the surface pointed to by pExtSurface.
        After a call to this, you should either set that pointer to NULL,
        if you don't need it anymore (DO NOT delete it), or refresh it with
        a call to GetExtendedSurface( pOriginalFace ).  The refreshed result
        may or may not be the same surface as was passed in, but it will be
        the proper, and only, extended surface for the face.

***********************************************************************/
void SmFilletCorner::AddExtendedOriginalSurface
 (SmFace    * pOriginalFace,  // in : face used as source of extSurface
  SmSurface * pExtSurface)    // in : an extended surface created from face->Surface
{
  // Check whether this face already has an extended surface registered.
  SmMapPtrToPtr<SmFace, SmSurface> & rMap = m_pExecutive->m_vExtendedSurfacesMap;
  SmSurface * pOldSurface = rMap.At(pOriginalFace);

  if (pOldSurface)
    {
      // We've already registered an extended surface for this face.
      // Keep the one with the larger domain, and delete the other.
      SmExtent2d sNewDomain = pExtSurface->GetNaturalUVDomain();
      SmExtent2d sOldDomain = pOldSurface->GetNaturalUVDomain();
      if ( sNewDomain.IsContainedBy( sOldDomain, SM_EFF_ZERO ) )
        {
          // Old domain was bigger, delete the new ext surface.
          SM_ASSERT(pExtSurface != NULL) ; delete pExtSurface ; pExtSurface = NULL ;
          return;
        }
      else
        {
          SM_ASSERT(pOldSurface != NULL) ; delete pOldSurface ; pOldSurface = NULL ;
        }
    }

  // add the OriginalFace/ExtSurface pair to the m_vExtendedSurfacesMap map list
  rMap.Insert( pOriginalFace, pExtSurface );

  return;

} // end SmFilletCorner::AddExtendedOriginalSurface

/*******************************************************************//**
PURPOSE: Create and return a derived SmFilletCorner whose Type
         is selected by the total number of edges connected to this vertex
         and how many of those edges are being filleted.

NOTES:
         Selects the fillet corner tolerances
           ApproximationTol = Min(All FilSolver->GetApproximationTols)
           TangencyTole     = Min(All FilSolver->GetTangencyTolerances)
         Also determines if the fillet is
           an OpenEndCorner,
           a  ChamferCorner.
***********************************************************************/
SmFilletCorner * SmFilletCorner::Create
 (const SmContext      & crContext,     // in : context for new object construction
  const SmVertex       * cpVertex,      // in : target vertex to fillet
  SmTArray<SmEdgeuse*> * pSolverEUs,    // in : array of edgeuses connected to vertex that are to be filleted
  SmFilletExecutive    * pExec)         // in : the FilletExecutive managing this filleting operations
{
  // init output
  SmFilletCorner * pNewCorner = NULL;

  // get number of edgeuses connected to this vertex to be filleted
  ULONG lTotalSolverEUs = pSolverEUs->GetSize();

  // locals
  SmBoolean bIsOpenEndCorner = FALSE;
  SmBoolean bIsChamferCorner = TRUE;
  ULONG ii;

  // Create array of all outward edgeuses connected to unique edges that surround the corner
  SmTArray<SmEdgeuse*> * pAllEUs = new(crContext) SmTArray<SmEdgeuse*>(crContext);

  // get first to-be-filleted edgeuse connected to vertex
  SmEdgeuse * pStartEU = (*pSolverEUs)[0];

static int iDebugLevel = 0; // Note: Can't call DebugLevel() from a static method.
#ifdef SM_DEBUG_CODE
  if ( iDebugLevel > 8 ) 
    {
      smgfx_Erase();
      smgfx_SetLook( 1,2, 0,0,0 ); pStartEU->GetBrep()->Draw(TRUE); sm_GraphicsLoop();
      smgfx_SetLook( 2,4, 0,1,0 ); pStartEU->GetFace()->Draw(); sm_GraphicsLoop();
      smgfx_SetLook( 4,6, 0,0,1 );
      for ( ULONG jkl=0; jkl<pSolverEUs->GetSize(); jkl++ ) 
        { (*pSolverEUs)[jkl]->Draw(); sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
      sm_GraphicsLoop();
    }
  if ( iDebugLevel > 0 ) 
    { smgfx_SetLook(4,8, 1,0,0); cpVertex->Draw(); sm_GraphicsLoop();
      pStartEU->Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  pAllEUs->Add(pStartEU);
  SmEdgeuse * pEU = pStartEU;
  while (TRUE)
    {
      // get edgeuse neighbors circuling this vertex and march to next radial neighbor
      SmEdgeuse * pEU0 = pEU;                  // edgeuse starting at vertex
      SmEdgeuse * pEU1 = pEU0->GetCWEdgeuse(); // edgeuse in same loop ending at vertex
      SmEdgeuse * pEU2 = pEU1->GetRadial();    // next edgeuse starting at vertex in same region
      pEU = pEU2;

#ifdef SM_DEBUG_CODE
      if ( iDebugLevel ) 
        {
          smgfx_ChangeColor(TRUE);
          smgfx_SetLineWidth(4.0);
          pEU1->Draw(); sm_GraphicsLoop();
          pEU2->Draw(); sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE

      // quit - when we get back to StartEU
      if (pEU == pStartEU) 
        { break; }

      // skip strut edgeuses
      if (pEU0->GetEdge() == pEU1->GetEdge()) 
        { continue; }

      // skip edgeuses that duplicate an edge - happens on closed curves
      SmBoolean bFound = FALSE;
      for (ii=0; ii<pAllEUs->GetSize(); ii++)
        {
          SmEdgeuse * pEU3 = (*pAllEUs)[ii];
          if (pEU->GetEdge() == pEU3->GetEdge())
            {
              bFound = TRUE;
              break;
            }
        }
      if (bFound) 
        { continue ; }

      // lamina edges have OpenEndCorners
      if (pEU->GetEdge()->IsLamina())
        {
          bIsOpenEndCorner = TRUE;
        }

      // If we have mixed convexity, pEU may not match the edgeuse in
      // pSolverEUs, use the one in pSolverEUs instead
      SmEdgeuse * pSolverEU = NULL;
      for (ii=0; ii<lTotalSolverEUs; ii++)
        {
          pSolverEU = (*pSolverEUs)[ii];
          if (   pEU != pSolverEU
              && pEU->GetEdge() == pSolverEU->GetEdge())
            {
              SM_DBG_WARN(_T("SmFilletCorner::Create(): CONFUSED CLASSIFICATION OF CORNER VERTEX - should be examined")) ;
              break;//mixed convexity
            }
          pSolverEU = NULL;
        }

      if (pSolverEU) { pAllEUs->Add(pSolverEU); }
      else           { pAllEUs->Add(pEU); }

    } // end while(TRUE) adding outward edgeuses to pAllEUs list

  // get total number of outward edgeuses connected to unique edges that are connected to vertex
  ULONG lTotalEUs = pAllEUs->GetSize();

  // get tightest tolerance of all to-be-filleted edgeuses connected to this vertex

  // 3D tolerance for corner patch creation
  double dApproxTol3d = SM_BIG_DOUBLE;

  // Angle tol for tangent field approximation
  double dTangencyTolRadians = SM_BIG_DOUBLE;

  // for every to-be-filleted edgeuse connected to vertex:
  // Collect max tolerances and check for Chamfer corner.
  for (ii=0; ii<lTotalSolverEUs; ii++)
    {
      // get edgeuse->edge FilletSolver and tolerance values
      SmEdgeuse      * pEdgeUse   = (*pSolverEUs)[ii];
      SmFilletSolver * pFilSolver = pExec->GetFilletSolverOfEdgeuse( pEdgeUse );
      double           dTol       = pFilSolver->GetThisApproxTol3d();
      double           dTanTol    = pFilSolver->GetTangencyTolerance();

      // save tightest tolerance value
      if ( dTol    < dApproxTol3d)         { dApproxTol3d = dTol;           }
      if ( dTanTol < dTangencyTolRadians ) { dTangencyTolRadians = dTanTol; }

      // when any to-be-filleted edgeuse is not linear - this it not a ChamferCorner
      if (   bIsChamferCorner == TRUE
          && pFilSolver->m_pFSG->GetFilletSurfaceGeneratorType() != SM_FSG_LINEAR)
        {
          bIsChamferCorner = FALSE;
        }
    } // end iter every to-be-filleted edgeuse connected to vertex

  double dTanTolDeg = SM_RAD2DEG( dTangencyTolRadians );

  // switch on number of to-be-filleted edgeuses connected to vertex
  switch (lTotalSolverEUs)
    {
      case 0:
          break;
      case 1: // one SolverEU
          {
              SmEdgeuse * pEU0  = (*pSolverEUs)[0];
              SmEdge    * pEdge = pEU0->GetEdge();
              if (pEdge->IsClosed())
                {
                  // Corner with one 'closed' edge filleted
                  switch (lTotalEUs)
                    {
                      case 1: // Need to be G1-closed
                          if (sm_TestG1Continuity(cpVertex,pEdge,pEdge, dTanTolDeg,NULL, iDebugLevel )) 
                            {
                              pNewCorner = new(crContext) SmFillet1x1Corner(crContext,
                                                                            cpVertex,
                                                                            pSolverEUs,
                                                                            pAllEUs,
                                                                            pExec,
                                                                            dApproxTol3d,
                                                                            dTangencyTolRadians);
                            }
                          else 
                            {
                              MSG(_T("Corner Case Not Implemented"));
                            }
                          break;
                      case 2:
                      case 3:
                          pNewCorner = new(crContext) SmFilletNx1ClosedCorner(crContext,
                                                                              cpVertex,
                                                                              pSolverEUs,
                                                                              pAllEUs,pExec,
                                                                              dApproxTol3d,
                                                                              dTangencyTolRadians);
                          break;
                      default:
                          MSG(_T("Corner Case Not Implemented"));
                    } // end switch on total number of edgeuses connected to vertex
                  break;
                }
              // Degeneracy-check needs to be checked first
              if (SmFilletNx1Corner::CheckNx1Degeneracy(cpVertex,pEU0)) 
                {
                  pNewCorner = new(crContext) SmFilletDegenerateCorner(crContext,
                                                                       cpVertex,
                                                                       pSolverEUs,
                                                                       pAllEUs,
                                                                       pExec,
                                                                       dApproxTol3d,
                                                                       dTangencyTolRadians);
                }
              else if (bIsOpenEndCorner) 
                {
                  pNewCorner = new(crContext) SmFilletOpenCorner(crContext,
                                                                 cpVertex,
                                                                 pSolverEUs,
                                                                 pAllEUs,
                                                                 pExec,
                                                                 dApproxTol3d,
                                                                 dTangencyTolRadians);
                }
              else 
                {
                  pNewCorner = new(crContext) SmFilletNx1Corner(crContext,
                                                                cpVertex,
                                                                pSolverEUs,
                                                                pAllEUs,
                                                                pExec,
                                                                dApproxTol3d,
                                                                dTangencyTolRadians);
                }
          }
          break ;

      case 2: // two SolverEU
          {
            SmEdgeuse * pEU0 = (*pSolverEUs)[0];
            SmEdgeuse * pEU1 = (*pSolverEUs)[1];
            switch (lTotalEUs) 
              {
                case 2: // 2x2: we can handle this only if it's a tangent fillet:
                    if ( sm_TestG1Continuity( cpVertex, pEU0->GetEdge(),
                        pEU1->GetEdge(),  dTanTolDeg, NULL, iDebugLevel ))
                      {
                        pNewCorner = new(crContext) SmFillet2x2Corner(crContext, 
                                                                      cpVertex, 
                                                                      pSolverEUs, 
                                                                      pAllEUs, 
                                                                      pExec,
                                                                      dApproxTol3d, 
                                                                      dTangencyTolRadians );
                      }
                    else 
                      {
                        MSG(_T("Corner Case Not Implemented"));
                      }
                    break;
                case 3:
                    if (    pEU0->IsConvexRadialSector(4)
                         != pEU1->IsConvexRadialSector(4) )
                      {
                        pNewCorner = new(crContext) SmFillet3x2MixedConvexityCorner(crContext,
                                                                                    cpVertex,
                                                                                    pSolverEUs,
                                                                                    pAllEUs,
                                                                                    pExec,
                                                                                    dApproxTol3d,
                                                                                    dTangencyTolRadians);
                        break;
                      }
                    // else continue with case 4
                case 4:
                    pNewCorner = new(crContext) SmFilletNx2Corner(crContext,
                                                                  cpVertex,
                                                                  pSolverEUs,
                                                                  pAllEUs,
                                                                  pExec,
                                                                  dApproxTol3d,
                                                                  dTangencyTolRadians);
                    break;
                default:
                    MSG(_T("Corner Case Not Implemented"));
              }
          }
          break;

      case 3: // three SolverEUs
          {
              if (lTotalEUs == 4) {
                  pNewCorner = new(crContext) SmFillet4x3Corner(crContext,
                                                                cpVertex,
                                                                pSolverEUs,
                                                                pAllEUs,
                                                                pExec,
                                                                dApproxTol3d,
                                                                dTangencyTolRadians);
                  break;
              }
          }
          // else continue with default case 
      default:
          {
            // Basically, we are handling NxN cases here
            if (lTotalEUs != lTotalSolverEUs) 
              {
                MSG(_T("Corner Case Not Implemented"));
                break;
              }
            SmTArray<SmEdgeuse*> sConcaveEUs;
            if (bIsChamferCorner == FALSE) 
              {
                sm_FindConcaveEdgeuses(cpVertex,pAllEUs,sConcaveEUs, iDebugLevel );
              }

            if (sConcaveEUs.GetSize() == 0) 
              {
                SmFilletConvexNxNCorner *pConvexNxNCorner = new(crContext)
                                                            SmFilletConvexNxNCorner(crContext,
                                                                                    cpVertex,
                                                                                    pSolverEUs,
                                                                                    pAllEUs,
                                                                                    pExec,
                                                                                    dApproxTol3d,
                                                                                    dTangencyTolRadians);
                pConvexNxNCorner->SetChamfer(bIsChamferCorner);
                pNewCorner = pConvexNxNCorner;
              }
            else 
              {
                pNewCorner = new(crContext) SmFilletConcaveNxNCorner(crContext,
                                                                     cpVertex,
                                                                     pSolverEUs,
                                                                     pAllEUs,
                                                                     pExec,
                                                                     dApproxTol3d,
                                                                     dTangencyTolRadians);
              }
          }
          break;
    } // end switch on

  return pNewCorner;

} // end SmFilletCorner::Create

/*******************************************************************//**
PURPOSE: Calculate geometry of all fillet vertices of this fillet corner.

NOTES:
***********************************************************************/
SmStatus SmFilletCorner::CalcCornerVertGeom()
{
  ULONG lNumVerts = m_vVertices.GetSize();
  SmTArray< SmBoolean > sAlreadyProcessed;
  sAlreadyProcessed.InsertAt( 0, FALSE, lNumVerts );
  ULONG ii, jj;

  // CalcCornerVertGeom() for every FilletVertex.
  for (ii=0; ii<lNumVerts; ii++)
    {
      SmFilletVertex * pCurrFV = m_vVertices[ii];  NER(pCurrFV);

      // Save this info for later.  [B570]
      sAlreadyProcessed[ii] = pCurrFV->IsProcessed();

      // switch on FilletVertex->m_eType to call appropriate vertex calculator
      SmStatus sStatus = pCurrFV->CalcCornerVertGeom();

#ifdef SM_DEBUG_CODE
      // draw origVertex(blue) and current FilletVertex(red)
      if ( DebugLevel() > 0 ) 
        {
          smgfx_SetLook(1,7, 0,0,1); m_cpVertex->Draw(); sm_GraphicsLoop();
          if (sStatus == SM_SUCCESS) { smgfx_SetLook(1,10, 1,0,0);
                                       pCurrFV->Draw(); sm_GraphicsLoop();
                                     }
          sm_GraphicsLoop();
      }
#endif // SM_DEBUG_CODE

      // for errors
      if (sStatus != SM_SUCCESS)
        {
    //      SmFilStatus eStatus = pCurrFV->GetStatus();
    //      this->InterpreteStatus(eStatus);
          SER(SM_ERR);
        }
    } // end iter all filletVertices

  // Now we need to adjust the FilletSolvers' LawIntervals, which map
  // the variable-radius functions to the Edge intervals.  [B551 B570]
  // Need to find them all and use the shortest interval.  [B663]

  // We do this only if the original was calculated as a constant-radius blend.
  SmBoolean bAdjustLawIntervals = ( this->IsCalcAsConstRad() );

  if ( ! bAdjustLawIntervals )
    { return SM_SUCCESS; }

  SmTArray< SmFilletGeom* > sFGs;

  // Get the corner vertex, for determining Start or End.
  const SmVertex * cpCornerVertex = this->GetFilletedVertex();

  for (ii=0; ii<lNumVerts; ii++)
    {
      if ( sAlreadyProcessed[ii] )
        { continue; }  // Would happen with a Mate, for example.

      SmFilletVertex * pCurrFV = m_vVertices[ii];  NER(pCurrFV);

      pCurrFV->GetFilletGeoms( sFGs );
      for ( jj=0; jj<sFGs.GetSize(); jj++ )
        {
          SmFilletGeom *pThisFG = sFGs[jj];

          SmFilletVertexuse *pFVU = pCurrFV->GetVUAtRailEnd( pThisFG );
          double dEdgeT = pFVU->GetTsectPnt().m_adUserDoubles[0];

          SmFilletSolver *pThisFS = pThisFG->GetFilletSolver();

          if ( ! pThisFS->OffsetRadiiCanChange() )
            { continue; }

          SmVariableRadiusFS * pVRFS = SM_CAST_PTR( SmVariableRadiusFS, pThisFS );
          if ( pVRFS == NULL )
            { continue; }

          SmExtent1d sCurrIvl = pVRFS->GetFilletLawInterval();
          SmExtent1d sNewIvl( sCurrIvl );

          // See which end.
          SmEdgeuse *pEU = pThisFS->GetEdgeuse(0);  NER( pEU );
          SmEdge *pFilletedEdge = pEU->GetEdge();
          SmBoolean bAtFilletStart = ( pFilletedEdge->GetStartVertex() == cpCornerVertex );
          SmBoolean bAtFilletEnd   = ( pFilletedEdge->GetEndVertex()   == cpCornerVertex );

          // A closed fillet edge needs the whole FilletLaw [B679]
          if ( bAtFilletStart && bAtFilletEnd )
            { break; }

          SmBoolean bAtLawStart = ( pVRFS->GetFilletLawOrientation() == FALSE )
                                      ?   bAtFilletStart
                                      : ! bAtFilletStart;

          // See whether this end of this FS should be adjusted: it should not be
          // adjusted if it's interior to the fillet radius law function.
          SmExtent1d sEdgeMap = pVRFS->GetFilletLaw()->m_vEdgeMap;
          double dLawParam = ( bAtLawStart ) ? sEdgeMap.GetMin() : sEdgeMap.GetMax();
          // The entire EdgeMap is always parameterizated 0 to 1.
          if ( dLawParam > SM_EFF_ZERO && dLawParam < 1.0-SM_EFF_ZERO )
            { continue; }

          // Set sNewIvl to a single point, at the other end, prior to adding new value.
          if ( bAtFilletStart )
            { sNewIvl.SetMinMax( sNewIvl.GetMax(), sNewIvl.GetMax() ); }
          else
            { sNewIvl.SetMinMax( sNewIvl.GetMin(), sNewIvl.GetMin() ); }

          sNewIvl.AddValue( dEdgeT );

          sNewIvl.Intersect( sCurrIvl, sNewIvl ); // Use the shortest interval.  [B663]

          // Check whether anything changes.
          SmExtent1d sCurrLawIvl( pVRFS->GetFilletLawInterval() );
          if ( ! ( sNewIvl == sCurrLawIvl ) )
            {
              pVRFS->SetFilletLawInterval( sNewIvl );
            }

        } // end for each FilletGeom connected to this FilletVertex
    } // end for each FilletVertex

  return SM_SUCCESS;

} // end SmFilletCorner::CalcCornerVertGeom

/*******************************************************************//**
PURPOSE: Calculate geometry of all corner edges (SmFilletEdge)

NOTES: Corner edges are the edges inserted at fillet corners to
  terminate fillet surfaces.  Corner edges are not the rail curves.

  This function is called after
    1. filletCorner->FilletVertex->Positions are set
    2. FilletSolver->FilletSurface->Shape is set
    3. FilletSolver->FilletRails->Shape are set
***********************************************************************/
SmStatus SmFilletCorner::CalcCornerEdgeGeom()
{
  // for every filletEdge (not a rail) ordered around this corner
  ULONG ii;
  for (ii=0; ii<m_vEdges.GetSize(); ii++)
    {
      SmFilletEdge * pCurrEdge = m_vEdges[ii];

      // skip deleted edges (possibly from degenerate corners)
      if ( pCurrEdge == NULL )
        { continue; }

      // Branch on SmFilletEdge->m_eType to pass call to
      // the appropriate edgeBuilding function

      if ( pCurrEdge->CalcCornerEdgeGeom() != SM_SUCCESS )
        {
          // CalcCornerEdgeGeom() failed

          // Error out if m_bDoGlobalMerge is FALSE or the type of edge is not
          // qualify for global merge afterwards.
          // NOTE: Basically, global Merge is to handle the cases
          // where fillets failed to intersect with a side face

          if (   !m_pExecutive->m_bDoGlobalMerge
              || pCurrEdge->GetFilletEdgeType() != SM_FE_FILLET_X_SIDE_FACE )
            {
              //SmFilStatus eStatus = pCurrEdge->GetStatus();
              SER(SM_ERR);
            }
          SmBrep *pBrep = pCurrEdge->GetBrep();
          SER(pBrep->DeleteEdge(pCurrEdge));
          m_vEdges[ii] = NULL;
        }
      else // pCurrEdge->CalcCornerEdgeGeom() succeeded
        {
          if ( pCurrEdge->GetStatus() == SM_FE_TO_BE_SQUEEZED )
            {
              // squeeze and delete it
              SER( SqueezeEdge(pCurrEdge) );
              continue;
            }

          // set currEdge->Interval = currEdge->Curve->NaturalInterval
          SmCurve    * pCurve = pCurrEdge->GetCurve(); NER(pCurve);
          SmExtent1d   sIvl   = pCurve->GetNaturalInterval();
          pCurrEdge->SetInterval(sIvl);

          // increment status to SM_FIL_PROCESSED for all but
          // SM_FE_CUBIC_RAIL_RAIL_INTERPOLATE cases
          if (pCurrEdge->GetStatus() != SM_FE_CUBIC_RAIL_RAIL_INTERPOLATE)
            {
              pCurrEdge->SetStatus(SM_FIL_PROCESSED);
            }

#ifdef SM_DEBUG_CODE
          if ( DebugLevel() > 0 ) 
            {
              smgfx_SetLook( 3,4, 1,0,0 ); pCurve->Draw(); sm_GraphicsLoop();
              sm_GraphicsLoop();
            }
#endif // SM_DEBUG_CODE
        } // end pCurrEdge->CalcCornerEdgeGeom() succeeded branch
    } // end iter every filletCorner->Edge (not rails)

  // remove all NULL entries from m_vEdges and m_vVertices
  m_vEdges.CompressZeros();
  m_vVertices.CompressZeros();

  // all done
  return SM_SUCCESS;

} // end SmFilletCorner::CalcCornerEdgeGeom

/*******************************************************************//**
PURPOSE: Calculate geometry of corner surface (only one surface for defalt corner)

NOTES: Any new FilletSurface created gets stored in m_vSurfaces
***********************************************************************/
SmStatus SmFilletCorner::CalcCornerGeom()
{
  ULONG lTotalEdges  = m_vEdges.GetSize();
  ULONG lFilletEdges = ( m_pSolverEUs == NULL ) ? 0 : m_pSolverEUs->GetSize();

  // Don't create a corner patch if m_bBevel: fillet surfaces are run long
  // and intersected with each other.
  // But if there is only one filleted edge at this corner, then there's nothing to
  // 'bevel' it with (no other fillet to intersect with), so go ahead and create
  // a corner patch if indicated.   [Fillet iter: 124]

  // no work - Bevel corner or degenerate corner
  if(  ( m_bBevel && lFilletEdges > 1 )
     ||  lTotalEdges < 3)  // Corner patches are not defined
    { return SM_SUCCESS; }

  // Otherwise, pass the call along to build a Corner->FilletSurface and store it in m_vSurfaces
  SER(CreateCornerPatch());

  return SM_SUCCESS;

} // end SmFilletCorner::CalcCornerGeom

/*******************************************************************//**
PURPOSE: Adjust the corner topology for those cases where all rails
    'meet' at the corner vertex, i.e. Degenerate corners.

NOTES:
    return FALSE when corner is not degenerate
    return TRUE  when corner is degenerate after deleting all
                   unnecessary topology objects and setting
                   m_bDegenerate = TRUE.

  degenerate corner := all corner filletVertices are within
                       m_dThisApproxTol3d of origVertex point

METHOD ---
  When a corner is degenerate - set m_vVertices->GetSize() == 1
                                set m_vEdges->GetSize() == 0
    1. delete all corner FilletEdges
    2. combine all corner FilletVertices
        a. move all FilletVertex->Vertexuses to 1st FilletVertex
        b. delete all but 1st FilletVertex
    3. update every vertexuse->TsectPoint->UVPoint position to be computed
       from the same common vertex position
    4. set m_bDegenerate = TRUE, return TRUE.
  Else return FALSE - no changes to internal structures
***********************************************************************/
SmBoolean SmFilletCorner::CheckAndFixDegeneracy()
{
  // get original vertex point location
  SmPoint3d sCornerPnt = m_cpVertex->GetPoint();

  // for every corner filletVertex
  ULONG ii, jj;
  for (ii=0; ii<m_vVertices.GetSize(); ii++)
    {
      SmFilletVertex * pFV = m_vVertices[ii]; NER(pFV);

      // low work - corners with unprocessed vertices are not degenerate
      if (!pFV->IsProcessed())
        { return FALSE; }

      // get corner filletVertex point
      SmPoint3d sPnt = pFV->GetPoint();

      // when filletVertex point is more than tolerance from origVertex point
      if (sPnt.DistanceBetween(sCornerPnt) > m_dThisApproxTol3d)
        {
          // the corner is not degenerat
          return FALSE;
        }

#ifdef SM_DEBUG_CODE
      if ( DebugLevel() > 0 ) 
        {
          smgfx_SetLook(4,10, 1,1,0); sPnt.Draw(); sm_GraphicsLoop();
          smgfx_SetLook(4,10, 0,0,0); sCornerPnt.Draw(); sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE
    }  // end iter all corner filletVertices

  // arrive here when all corner filletVertices are within tolerance
  // of origVertex point

  // delete all corner edges (not rails)
  for (ii=0; ii<m_vEdges.GetSize(); ii++)
    {
      SmFilletEdge * pFilletEdge = m_vEdges[ii];
      m_pPseudoBrep->RemoveEdge(pFilletEdge);
      m_pExecutive->RemoveFromTopoEdges(pFilletEdge) ;
      SM_ASSERT(pFilletEdge != NULL) ; delete pFilletEdge ; m_vEdges[ii] = NULL ;
    }
  m_vEdges.RemoveAll();

  // Combine all vertices into the 1st corner FilletVertex

  // get the 1st corner FilletVertex
  SmFilletVertex * pFV0 = m_vVertices[0];

  // for every corner FilletVertex except the first
  for (ii=1; ii<m_vVertices.GetSize(); ii++)
    {
      SmFilletVertex * pFV = m_vVertices[ii]; NER(pFV);

      // move every cornerVertex->Vertexuse to 1st FilletVertex
      SmTArray<SmVertexuse*> sVertexuses;
      pFV->GetVertexuses(sVertexuses);
      for (jj=0; jj<sVertexuses.GetSize(); jj++)
        {
          SmFilletVertexuse * pVU = (SmFilletVertexuse*)sVertexuses[jj];

          // move vertexuse to 1st FilletVertex
          pFV->Remove(pVU);
          pFV0->PostInsert(pVU);

          // set associated TsectPnt type to SM_IP_SINGULARITY
          SmTsectPnt & rTsectPnt = pVU->GetTsectPnt();
          rTsectPnt.m_ePointType = SM_IP_SINGULARITY;
        }

      // delete the FilletVertex
      SM_ASSERT(pFV != NULL) ; delete pFV ; pFV = NULL ;

    } // end iter every corner filletVertex but the 1st

  // set m_vVertices to a list of 1 filletVertex
  m_vVertices.ReSet();
  m_vVertices.Add(pFV0);

  // set the filletVertex to classify to the orig Vertex
  pFV0->SetPointClass(SM_PC_VERTEX, (SmObject*)m_cpVertex);
  pFV0->SetFilletVertexType(SM_FV_ON_VERTEX);

  // get vertexuses for both origVertex and the remaining corner filletVertex
  SmTArray<SmVertexuse*> sOrigVUs;
  m_cpVertex->GetVertexuses(sOrigVUs);
  SmTArray<SmVertexuse*> sFilVUs;
  pFV0->GetVertexuses(sFilVUs);

  // for every corner FilletVertex->Vertexuse - set its TsectPnt->UVPoint position
  for (ii=0; ii<sFilVUs.GetSize(); ii++)
    {
      SmFilletVertexuse * pFilVU = (SmFilletVertexuse*)sFilVUs[ii];

      // filletVertexuse locals: TsectPnt, rail->face, railIndex
      SmTsectPnt      & rTsectPnt  = pFilVU->GetTsectPnt();
      SmFilletEdgeuse * pFilEU     = (SmFilletEdgeuse*)pFilVU->GetEdgeuse();
      SmFilletEdge    * pRail      = (SmFilletEdge*)pFilEU->GetEdge();
      SmFace          * pF         = pRail->GetOriginalFace(); NER(pF);
      SmFilletGeom    * pFG        = pFilEU->GetFilletGeom(); NER(pFG);
      SmFilletSolver  * pFilSolver = pFG->GetFilletSolver();
      ULONG             lRailIndex = pFilSolver->FindIndexOfRailOnFace(pF);

      // for every original vertex->Vertexuse
      for (jj=0; jj<sOrigVUs.GetSize(); jj++)
        {
          SmVertexuse * pVU = sOrigVUs[jj];

          // when filletVertexuse->Rail->Face == origVertexuse->Face
          if (pVU->GetFaceuse()->GetFace() == pF)
            {
              // refine the filletVertex->TsectPnt->UVpoint position
              SmPoint2d sUV;
              pVU->ComputeUVPoint(sUV);
              rTsectPnt.UVPos(lRailIndex) = sUV;
              break;
            }
        } // end iter every original vertex->Vertexuse
    } // end iter every corner FilletVertex->Vertexuse updating UVPoint values

  // inform the public - corner is degenerate
  m_bDegenerate = TRUE;
  return TRUE;

} // end SmFilletCorner::CheckAndFixDegeneracy

/*******************************************************************//**
PURPOSE: Calculate geometry of an end surface of Nx1 corner and create Face 
  (and Edges and Verts) in m_pFilletBrep from m_vSurfaces with call 
  m_pFilletBrep->MakeFaceWithCurves()

NOTES:
  Corner EndFaces are built using an extended Surface of one of 
  the filletCorners->Side faces (a face intersecting the fillet
  not one of the faces along which the fillet has been running.)

METHOD ---
  For the 1st FilletEdge found of type SM_FE_FILLET_X_SIDE_FACE
   1. Copy its FilletEdge->OriginalFace->ExtendedSurface and
      store that in m_vSurfaces array.
   2. Call MakeFaceBrep()
***********************************************************************/
SmStatus SmFilletCorner::CreateEndPatch()
{
  // for every filletCorner->FilletEdge
  ULONG ii, lNumEdges = m_vEdges.GetSize();
  for (ii=0; ii<lNumEdges; ii++)
    {
      SmFilletEdge * pCurrEdge = m_vEdges[ii]; NER(pCurrEdge);

      // skip non SM_FE_FILLET_X_SIDE_FACE FilletEdges
      if (pCurrEdge->GetFilletEdgeType() != SM_FE_FILLET_X_SIDE_FACE)
        {
          continue;
        }

      // Get FilletEdge->OriginalFace->ExtendedSurface or Surface when needed
      SmFace * pFace = pCurrEdge->GetOriginalFace();
      NER(pFace);
      SmSurface * pSurface = GetExtendedSurface(pFace);
      if (pSurface == NULL)
        {
          pSurface = pFace->GetSurface();
        }

      // gwc:Change needed here - this surface needs to come from
      //   either this surface or a g0 cap surface

      // Add a copy of this surface to the FilletCorner->m_vSurfaces array
      SmSurface * pNewSurface = NULL;

      // Copy pSurface, when possible as an analytic surface
      SER(pSurface->CopyAndAddAnalytics(m_crContext,pNewSurface));
      m_vSurfaces.Add(pNewSurface);

#ifdef SM_DEBUG_CODE
  // draw CurrentEdge(Blue), OriginalFace(red)
  if( DebugLevel() > 0 )
    {
      smgfx_SetLook(4,6, 0,0,1) ; pCurrEdge->Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(4,6, 1,0,0) ; pFace->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

      // Create Face (and Edges and Verts) in m_pFilletBrep from m_vSurfaces with call m_pFilletBrep->MakeFaceWithCurves()
      if(SM_SUCCESS != MakeFaceBrep())
        {
          // when MakeFaceBrep() failed - Note FilletError
          TCHAR sBuff[SM_TBLOCK_SIZE];
          this->GetName( sBuff, SM_TBLOCK_SIZE );
          GetFilletExecutive()->NoteFilletError( SM_FILERR_VERTEX_PROBLEM,
                                                 this->GetCornerType(), SM_FIL_FAILURE, sBuff, this->GetFilletedVertex(),
                                                 NULL, NULL, // For now, we don't generally have just two surfaces.
                                                 NULL, NULL, // For now, we don't generally have just two edgeuses.
                                                 _T("Fillet Corner Error: Corner End Face Creation Error") );
          FILEXEC_CORNER_ERR(this,_T("Corner Face Creation Error"));
        }

      break;
    } // end iter every filletEdge searching for one attached to face to extend

  // all done
  return SM_SUCCESS;

} // end SmFilletCorner::CreateEndPatch

/*******************************************************************//**
PURPOSE: Calculate geometry of a 3- or 4-sided COONS patch or N-sided
         patchesa and create Face (and Edges and Verts) in m_pFilletBrep 
         from m_vSurfaces with call MakeFaceBrep().
  
NOTES: For the 3-sided case, a degenerate 'point' curve will be
    passed into COONS creation routine.
***********************************************************************/
SmStatus SmFilletCorner::CreateCornerPatch()
{
  ULONG ii;
  // if an AnalyticSurface can approx FilletCorner->Curves to tol, 
  //   Make and store that AnalyticSurface in m_vSurfaces, and return TRUE
  // else return FALSE.
  SmBoolean bSuccess = MakeAnalyticCornerSurface();

  if( ! bSuccess )
    {
      // When unable to create analytic corner - create COONS patch

      ULONG            lInitIndex           = 0;
      ULONG            lTotalEdges          = m_vEdges.GetSize();
      SmFilletVertex * pFVOfDegenerateCurve = NULL;  // used to indicate degenerate Curve of 3 sided patches

      // for corners with 3 edges - select a degenerate vertex (lInitIndex)
      if (lTotalEdges == 3)
        {
          // Select degenerate corner - Pay special attention to Nx2 corners
          enum SmFilletCornerType eCornerType = GetCornerType();
          if (   eCornerType == SM_FCR_N_x_2
              || eCornerType == SM_FCR_3_x_2_MIXED
              || eCornerType == SM_FCR_N_x_1)
            {
              // for every corner vertex
              for (ii=0; ii<m_vVertices.GetSize(); ii++)
                {
                  SmFilletVertex * pFV = m_vVertices[ii];

                  // switch on corner->VertexType
                  switch (pFV->GetFilletVertexType())
                    {
                      case SM_FV_RAIL_X_EDGEUSE   : if (pFVOfDegenerateCurve == NULL)
                                                      { pFVOfDegenerateCurve = pFV; }
                                                    break;

                      case SM_FV_FILLET_X_FILLET  : pFVOfDegenerateCurve = pFV;
                                                    break;

                      case SM_FV_ON_VERTEX        : if(eCornerType == SM_FCR_N_x_1)
                                                      { pFVOfDegenerateCurve = pFV; }
                                                    break ;

                      case SM_FV_FILLET_X_EDGEUSE :
                      case SM_FV_FILLET_X2_FILLETS:
                      case SM_FV_MATE             :
                      case SM_FV_ON_CROSS_SECTION :
                      case SM_FV_CLIFF_RAIL_X_EDGE:
                      case SM_FV_RAIL_END         :
                      case SM_FV_RAIL_X_RAIL      :
                      case SM_FV_RAIL_X_VERTEX    :
                      case SM_FV_RAIL_X_EXTENDED_EDGEUSE:
                      case SM_FV_SETBACK          :
                      case SM_FV_UNKNOWN          : break;

                    } // end switch on corner->VertexType
                } // end iter every corner vertex
            } // end SM_FCR_N_x_2 or SM_FCR_3_x_2_MIXED corner type check

          // when we have yet to select the degenerate corner
          if (pFVOfDegenerateCurve == NULL)
            {
              // get distance between every filletCorner->FilletVertex pair
              double dPntPntDist[4];
              for (ii=0; ii<3; ii++)
                {
                  ULONG lNextVert = (ii+1)%3 ;
                  SmPoint3d sPnt  = m_vVertices[ii]->GetPoint();
                  dPntPntDist[ii] = sPnt.DistanceBetween(m_vVertices[lNextVert]->GetPoint());
                }
              dPntPntDist[3] = dPntPntDist[0];

              // select degenerate vertex = vertex connected to edges most equal in length
              ULONG lVertOfDegCurve = 0;
              double dMinDiff = smos_Fabs(dPntPntDist[2] - dPntPntDist[0]);
              for (ii=1; ii<3; ii++)
                {
                  double dDiff = smos_Fabs(dPntPntDist[ii-1] - dPntPntDist[ii]);
                  if (dDiff < dMinDiff)
                    {
                      dMinDiff = dDiff;
                      lVertOfDegCurve = ii;
                    }
                }

              // set the Degenerate vertex pointer
              pFVOfDegenerateCurve = m_vVertices[lVertOfDegCurve];

            } // end need to select pFVOfDegenerateCurve check

#ifdef SM_DEBUG_CODE
          if ( DebugLevel() > 0 ) 
            {
              smgfx_SetLook(4,6, 1,0,0); pFVOfDegenerateCurve->Draw(); sm_GraphicsLoop();
              sm_GraphicsLoop();
            }
#endif // SM_DEBUG_CODE
          // get index of degenerate vertex
          for (ii=0; ii<3; ii++)
            {
              if (m_vEdges[ii]->GetVertex() == pFVOfDegenerateCurve)
                {
                  lInitIndex = ii;//Left curve index
                  break;
                }
            }
        } // end corner has 3 edges - so set pFVOfDegenerateCurve and lInitIndex values - branch


      // else for corners with 4 edges branch - set lInitIndex == EdgeOfType SM_FE_RAIL_RAIL_INTERPOLATION
      else if (lTotalEdges == 4)
        {
          for (ii=0; ii<lTotalEdges; ii++)
            {
              SmFilletEdge * pFE = m_vEdges[ii];
              if (pFE->GetFilletEdgeType() == SM_FE_RAIL_RAIL_INTERPOLATION)
                {
                  lInitIndex = (ii+1)%lTotalEdges;//Left curve index
                  break;
                }
            }
        } // end corner has 4 edges branch

      // arrive here once lInitIndex has been set

      // Next - build ordered patch boundary arrays.  Fill in the following locals:
      SmTArray<SmBSplineCurve*> sOrderedUV;
      SmTArray<SmCurve*>        sOrdered3D;
      SmTArray<SmOrientType>    sOrients;
      SmTArray<SmSurface*>      sOrderedSurfs;
                                
      // for every edge - get edge shape as (neighborFace->Surface,UVTrimCurve) pair
      for (ii=0; ii<lTotalEdges; ii++)
        {
          SmFilletEdge    * pFE          = m_vEdges[(lInitIndex+ii)%lTotalEdges];
          SmBSplineCurve  * pPSCurve     = NULL;
          SmSurface       * pBaseSurface = NULL;
          SmFilletEdgeuse * pPrimEU      = (SmFilletEdgeuse*)pFE->GetPrimaryEdgeuse();
          SmFilletEdgeuse * pMateEU      = (SmFilletEdgeuse*)pPrimEU->GetMate();

          // when build g1 patches - get UVTrimCurve and Surface
          if(m_bExtendedSurfacePatch)
            {
              // Get pPSCurve and pBaseSurface.

              // when the edge has an OriginalFace
              if (pFE->GetOriginalFace())
                {
                  pPSCurve     = pMateEU->GetUVTrimCurve();
                  pBaseSurface = pFE->GetOriginalFace()->GetSurface();
                }
              else // no Original Face branch - curve is only connected to filletSurfaces
                {
                  // use primaryEdgeuse to get (neighborFace,UVTrimCurve) pair
                  pPSCurve                   = pPrimEU->GetUVTrimCurve();
                  SmFilletGeom * pFilletGeom = pPrimEU->GetFilletGeom();
                  NER(pFilletGeom);
                  pBaseSurface = (SmSurface*)pFilletGeom->GetFilletSurface();
                }
            } // end getting pPSCurve and pBaseSurface

          //switch (pFE->GetFilletEdgeType())
          //case SM_FE_RAIL_RAIL_INTERPOLATION:
          //case SM_FE_SETBACK_RAIL:
          //    pPSCurve     = pMateEU->GetUVTrimCurve();
          //    pBaseSurface = pFE->GetOriginalFace()->GetSurface();
          //    break;
          //default:
          //    pPSCurve     = pPrimEU->GetUVTrimCurve();
          //    SmFilletGeom * pFilletGeom = pPrimEU->GetFilletGeom();
          //    NER(pFilletGeom);
          //    pBaseSurface = (SmSurface*)pFilletGeom->GetFilletSurface();
          //

          // add edge 3DCurve, NeighborSurface, NeighborSurface->UVTrimCurve, orients to ordered arrays
          sOrderedUV.Add(pPSCurve);
          sOrdered3D.Add(pFE->GetCurve());
          sOrients.Add(SM_OT_SAME);
          sOrderedSurfs.Add(pBaseSurface);

#ifdef SM_DEBUG_CODE
          if ( DebugLevel() > 0 ) 
            {
              if ( DebugLevel() > 10 && pBaseSurface != NULL )
              {
                smgfx_SetLook(1,2, 1,0,0); pBaseSurface->DrawUV(0,0); sm_GraphicsLoop();
                sm_GraphicsLoop();
              }
              smgfx_SetLook(1,2, 0,0,1); pFE->GetCurve()->Draw(); sm_GraphicsLoop();
              sm_GraphicsLoop();
            }
#endif // SM_DEBUG_CODE
        } // end iter every edge - building ordered patch boundary arrays

      // arrive here after building the following odered arrays for edges bounding 3 and 4 sided corner patches
      //   sOrderedUV
      //   sOrdered3D
      //   sOrients
      //   sOrderedSurfs

      // next - standardize the ordered arrays for 3 and 4 sided patches by adding a degenerate curve for 3 sided patches

      // when building a 3 sided patch - add a degenerate curve to the Ordered arrays making them the same as for 4 sided patches
      SmBSplineCurve * pPointCurve   = NULL;
      SmBSplineCurve * pUVPointCurve = NULL;
      SmObjDelete sDelete1;
      SmObjDelete sDelete2;
      if (pFVOfDegenerateCurve)
        {
          // Append degenerate(TOP) curve for 3-sided patch

          // make a temporary degenerate 3DCurve
          SmPoint3d sPnt = pFVOfDegenerateCurve->GetPoint();
          SER(SmBSplineCurve::CreatePointCurve(m_crContext,          /* parameterized from 0 to 1 */
                                               sPnt,
                                               pPointCurve));
          sDelete2.SetObj(pPointCurve);

          // when degenerate vertex classifies to a Face
          // and building g1 patches - give it a (surface,UVTrimCurve) pair
          SmSurface * pBaseSurface = NULL;
          SmFace * pFace = SM_CAST_PTR(SmFace, pFVOfDegenerateCurve->GetPointClassObject());
          if (pFace && m_bExtendedSurfacePatch)
            {
              // Degenerate curve lies on a surface. (We'll not have a base surface
              //                                      if it's SM_FV_RAIL_X_EDGEUSE)
              pBaseSurface  = pFace->GetSurface();
              SmPoint2d sUV = pFVOfDegenerateCurve->GetOriginalUV();
              SmPoint3d sPt(sUV);
              SER(SmBSplineCurve::CreatePointCurve(m_crContext,        /* parameterized from 0 to 1 */
                                                  sPt,
                                                  pUVPointCurve));
              pUVPointCurve->ConvertTo2D() ; // gwc: converts z=0.0 to z=NL_NOZ values - should not be a problem
              sDelete1.SetObj(pUVPointCurve);
            }

          // add degenerate curve to ordered boundary arrays
          sOrderedUV.Add(pUVPointCurve);
          sOrdered3D.Add(pPointCurve);
          sOrients.Add(SM_OT_SAME);
          sOrderedSurfs.Add(pBaseSurface);
          lTotalEdges++;

        } // end 3 sided patch - so add a degenerate curve to ordered arrays - check

      // arrive here when all ordered arrays have 4 entries (for both 3 and 4 sided patches)

      // next - create the blend surface - if possible
      SmTArray<SmSurface*> sBlends;

      // build the corner patch from the ordered array boundary information
      if (sBlends.GetSize() == 0)
        {
          SER(SmBSplineSurface::CreateCornerBlend
                    (m_crContext,
                     sOrdered3D, 
                     sOrients, 
                     sOrderedUV, 
                     sOrderedSurfs,
                     m_dThisApproxTol3d,
                     m_dTangencyTolRadians,
                     sBlends,                           // out: blended surface set
                     m_bExtendedSurfacePatch ? 0 : 1));
        }

      // store New FilletSurface in m_vSurfaces
      m_vSurfaces.Append(sBlends);

#ifdef SM_DEBUG_CODE
      if ( DebugLevel() > 0 ) 
        {
          for (ULONG ij=0; ij<sBlends.GetSize(); ij++)
            {
              sBlends[ij]->DrawUV(4,4); sm_GraphicsLoop();
              sm_GraphicsLoop();
            }
        }
#endif // SM_DEBUG_CODE

    } // end can't create an analytic corner - so need to create a coons patch check

  // Create Face (and Edges and Verts) in m_pFilletBrep from m_vSurfaces with call m_pFilletBrep->MakeFaceWithCurves()
  SmStatus eStat = MakeFaceBrep();
  if ( eStat != SM_SUCCESS )
    {
      // when MakeFaceBrep failed - note the error
      TCHAR sBuff[SM_TBLOCK_SIZE];
      this->GetName( sBuff, SM_TBLOCK_SIZE );
      GetFilletExecutive()->NoteFilletError( SM_FILERR_VERTEX_PROBLEM,
                                             this->GetCornerType(), SM_FIL_FAILURE, sBuff, this->GetFilletedVertex(),
                                             NULL, NULL, // For now, we don't generally have just two surfaces.
                                             NULL, NULL, // For now, we don't generally have just two edgeuses.
                                             _T("Fillet Corner Error: Corner Face Creation Error") );
      FILEXEC_CORNER_ERR(this,_T("Corner Face Creation Error"));
    }

  // all done
  return SM_SUCCESS;

} // end SmFilletCorner::CreateCornerPatch

/*******************************************************************//**
PURPOSE: Delete (and eliminate) a vertex from the corner topology

NOTES: All corner edges that are connected to this vertex
    will also be deleted. One of the end vertices should have its
    status been set as SM_FV_TO_BE_DELETED
***********************************************************************/
SmStatus SmFilletCorner::SqueezeEdge
 (SmFilletEdge *pEdgeToSqueeze)
{
  // Get the vertex that will be deleted
  SmFilletVertex * pSurvivingFV = (SmFilletVertex*)pEdgeToSqueeze->GetVertex();
  SmFilletVertex * pDeleteFV    = (SmFilletVertex*)pEdgeToSqueeze->GetOtherVertex(pSurvivingFV);
  if (pDeleteFV->GetStatus() != SM_FV_TO_BE_DELETED) { SM_SWAP_PTR(SmFilletVertex,pSurvivingFV,pDeleteFV); }
  if (pDeleteFV->GetStatus() != SM_FV_TO_BE_DELETED) { SER(SM_ERR); }

  SmTArray<SmVertexuse*> sVUs;
  pDeleteFV->GetVertexuses(sVUs);
  ULONG ii, lFoundIndex;
  for (ii=0; ii<sVUs.GetSize(); ii++) 
    {
      SmFilletVertexuse * pVU = (SmFilletVertexuse*)sVUs[ii];
      SmFilletEdge * pFE = (SmFilletEdge*)pVU->GetEdgeuse()->GetEdge();
      if (pFE->GetFilletEdgeType() == SM_FE_RAIL) 
        {
          pDeleteFV->Remove(pVU);
          SER(pSurvivingFV->PostInsert(pVU));
        }
      else 
        {
          if (!m_vEdges.FindElement(pFE,lFoundIndex)) 
            { SER(SM_ERR); }

          m_vEdges[lFoundIndex] = NULL;
          //m_vEdges.SetAt(lFoundIndex);
          SM_ASSERT(pFE != NULL) ; delete pFE ; pFE = NULL ;
        }
    }
  if (!m_vVertices.FindElement(pDeleteFV,lFoundIndex)) 
    { SER(SM_ERR); }

  m_vVertices[lFoundIndex] = NULL;
  SM_ASSERT(pDeleteFV != NULL) ; delete pDeleteFV ; pDeleteFV = NULL ;

  // all done
  return SM_SUCCESS;

} // end SmFilletCorner::SqueezeEdge

/*******************************************************************//**
PURPOSE: Return an extended surface of a face of the original brep
            or NULL when there isn't one.

NOTES:
***********************************************************************/
SmSurface * SmFilletCorner::GetExtendedSurface
  (SmFace * pOriginalFace)
{
  SmMapPtrToPtr<SmFace, SmSurface> & rMap = m_pExecutive->m_vExtendedSurfacesMap;
  SmSurface * pExtSurface = rMap.At(pOriginalFace);

  return pExtSurface;

} // end SmFilletCorner::GetExtendedSurface

/*******************************************************************//**
PURPOSE: Insert the intersection topology of the fillet curves into the 
     original m_pTargetBrep and set up the m_vTI Topology Intersector Relationships.

NOTES: 
***********************************************************************/
SmStatus SmFilletCorner::InsertIntersectionTopology()
{
  // locals
  ULONG ii, jj;
  SmFilletExecutive        * pFilletExec = GetFilletExecutive();
  SmTopologyIntersector    & rTI = pFilletExec->GetTopologyIntersector();
  SmTArray<SmCurve*>         s3DCurves, sUVCurves1, sUVCurves2;
  SmTArray<double>           sDeviations;
  SmTArray<SmTsectCurveType> sCurveTypes;
  SmTArray<SmAObject*>       sAllEdges;
  SmTArray<SmEdge*>          sEdges;
   SmTArray<SmFace*>         sFEFaces;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe=FALSE;
  if ( bDebugMe ) 
    {
      smgfx_Erase();
      smgfx_SetLook( 1,2, 0,0,0 ); rTI.GetPrimaryBrep()->Draw(TRUE); sm_GraphicsLoop();
      smgfx_SetLook( 2,4, 0,1,0 ); rTI.GetOtherBrep  ()->Draw(TRUE); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // when asked
  if (m_bBlending)
    {
      SmTArray<SmFace*> sVertFaces;
      m_cpVertex->GetFaces(sVertFaces);
      for (ii=0; ii<sVertFaces.GetSize(); ii++)
        {
          SmFace *pVF = sVertFaces[ii];
          for (jj=0; jj<m_vSurfaces.GetSize(); jj++)
            {
              SmSurface *pFBSurface = m_vSurfaces[jj];
              SER(rTI.RegisterExistingSSI(pVF->GetSurface(),pFBSurface,s3DCurves,
                                          sUVCurves1,sUVCurves2,sCurveTypes,sDeviations));
            }
        }
    } // end m_bBlending == TRUE check

  // for every corner FilletEdge - Merge corner edge into originating face (ony edges with orig m_pTargetBrep Faces get merged here)
  for (ii=0; ii<m_vEdges.GetSize(); ii++)
    {
      SmFilletEdge * pFE = m_vEdges[ii];
      if ( pFE == NULL )
        { continue; }  // Can happen for some Corner Edges that aren't in the target Brep.

#ifdef SM_DEBUG_CODE
      if ( bDebugMe ) 
        {
          smgfx_SetLook( 3,5, 0,0,1 ); pFE->Draw(); sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE

      // skip faces not on OrigFaces
      SmFace *pFace = pFE->GetOriginalFace();
      if ( !pFace ) 
        { continue; }

#ifdef SM_DEBUG_CODE
      if ( bDebugMe ) 
        {
          smgfx_SetLook( 1,2, 0,1,1 ); pFace->DrawUV(3,3); sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE

      // get associated FilletEdge->BrepEdges (destined for m_pFilletBrep) that coincide with this m_pPseudoBrep->Edges (usually 1, can be 2 for split cases, Setbacks, etc.)
      pFE->GetFilletBrepEdges( sEdges );

      // For every FilletEdge->m_pFilletBrep->Edge
      for (jj=0; jj<sEdges.GetSize(); jj++)
        {
          SmEdge  * pEdge    = sEdges[jj];
          SmCurve * p3DCurve = pEdge ? pEdge->GetCurve() : NULL ;
          if ( !pEdge ) { continue; } // gwc: does this happen?

#ifdef SM_DEBUG_CODE
          if ( bDebugMe ) 
            {
              smgfx_SetLook( 5,7, 1,0,0 ); pEdge->Draw(); sm_GraphicsLoop();
              sm_GraphicsLoop();
            }
#endif // SM_DEBUG_CODE

          // skip pEdge that can't be found on the m_pAttributeE->Users list
          //cbi Couldn't we do this ahead of time?  Does the list change?
          pFilletExec->m_pAttributeE->GetUsers(sAllEdges);
          ULONG lIndex;
          if ( !sAllEdges.FindElement(pEdge,lIndex) )
            { continue; }

          // skip Edges that don't have just one face
          pEdge->GetFaces( sFEFaces );
          if (sFEFaces.GetSize() != 1) 
            { continue; } 

          SmFace         * pFilletBrepFace = sFEFaces[0];
          SmBSplineCurve * pUVCurve        = NULL;
          SmBoolean        bDeletedTopology;

          // Don't intersect the blending corner with the adjacent faces of
          // the vertex which is being blended.

          if (sEdges.GetSize() == 1) 
            {
              // pUVCurve = pFE->GetOriginalUVCurve();
              p3DCurve = pFE->GetCurve();
            }

          // Put an attribute on the base Face, to check whether it gets split.
          SmAttribute sFaceAttr( 780, SM_AB_STANDALONE_REFERENCE );
          pFace->AddAttribute( &sFaceAttr );

          // Merge a curve which is on two surfaces belonging to two faces of two different Breps.
          SER(rTI.MergeCurveOnSurfaces(pFace->GetSurface(),   // in : 1st surface of surface/surface intersection
                                       NULL,                  // in : 2nd surface of surface/surface intersection
                                       pEdge->GetTolerance(), // in : min length for non-degenerate curve. Degen Crv treated as pt.
                                       p3DCurve,              // in : surf/surf 3D xsect curve
                                       pUVCurve,              // in : opt associated UV curve for 1st surface
                                       NULL,                  // in : opt associated UV curve for 2nd surface
                                       NULL,                  // in : opt already existing edge corresponding to 3DCurve on pSurface,
                                                              //      NULL to ignore
                                       pEdge,                 // in : opt already existing edge which corresponding to 3DCurve on pOtherSurface,
                                                              //      NULL to ignore
                                       bDeletedTopology));    // out: TRUE = multiply mated Vertices or Edges squeezed in MergeCurveClasses().
                                                              //      FALSE= no Vertices or Edges squeezed
                                                              // out: ptrs to deleted edges
                                                              // out: ptrs to deleted vertices

          // If any Faces were split, check whether objects sitting on the
          // original Face are now in the new split Face.
          pFilletExec->UpdateTopologyChanges( &sFaceAttr, pFace );

          // Right now Just register no intersections
          SER( rTI.RegisterExistingSSI( pFace->GetSurface(),           // in : surf1
                                        pFilletBrepFace->GetSurface(), // in : surf2
                                        s3DCurves,                     // in : 3d xsect curves
                                        sUVCurves1,                    // in : associated surf1 UVTrimCurves
                                        sUVCurves2,                    // in : associated surf2 UVTrimCurves
                                        sCurveTypes,                   // in : associated intersection curve types
                                        sDeviations) );                // in : associated intersection curve deviations
#ifdef VALIDATE_POINTERS
          pFace->GetBrep()->ValidatePointers();
#endif // VALIDATE_POINTERS
        } // end iter every FilletEdge->m_pFilletBrep->Edge
    } // end iter every corner FilletEdge - Merge corner edge into originating face (ony edges with orig m_pTargetBrep Faces get merged here)

  // all done
  return SM_SUCCESS;

} // end SmFilletCorner::InsertIntersectionTopology

/*******************************************************************//**
PURPOSE: In the case where we have a 4-sided corner (3x3) with one
    interpolating edge, try to convert it to a variable-radius fillet
    to achive better G1 continuity. In the current filleting framework,
    two side

NOTES: Return SM_SUCCESS if successfully create a fillet surface
    for this corner. Otherwise, return SM_ERR.
***********************************************************************/
SmStatus SmFilletCorner::MakeVariableRadiusFillet
  // gwc: I think pSideSurface no longer has to be a BBSplineSurface - replace 1 line
  // rm : (SmBSplineSurface *& rpSurface)  // out: New Corner FilletSurface if Converted, else NULL
  (SM_FILLETSURF_TYPE *& rpSurface)  // out: New Corner FilletSurface if Converted, else NULL
{
  // init output
  rpSurface = NULL ;

  // locals
  ULONG lTotalEdges = m_vEdges.GetSize();

  // no work - corner does not have exactly 4 sides. This function only converts 4-sided corners
  if (lTotalEdges != 4) 
    { return SM_ERR; } // Unable to convert

  // Find surfaces of blend
  ULONG ii, lFoundIndex = 9999;
  for (ii=0; ii<m_vEdges.GetSize(); ii++) 
    {
      SmFilletEdge * pE = m_vEdges[ii];
      if (pE->GetFilletEdgeType() == SM_FE_RAIL_RAIL_INTERPOLATION) 
        {
          lFoundIndex = ii;
          break;
        }
    } // end iter all corner edges seeking FilletEdge of type SM_FE_RAIL_RAIL_INTERPOLATION 

  // no work - can't find a corner bounding edge of type SM_FE_RAIL_RAIL_INTERPOLATION
  if (lFoundIndex == 9999) 
    { return SM_ERR;  } // Unable to convert
    
  // locals
  SmFilletEdge   * pCrossEdges[2];
  SmFilletEdge   * pInterpolateEdge       = m_vEdges[lFoundIndex];
  SmFilletEdge   * pOppositeEdge          = m_vEdges[(lFoundIndex+2)%lTotalEdges];
  pCrossEdges[0]                          = m_vEdges[((lFoundIndex+lTotalEdges)-1)%lTotalEdges];
  pCrossEdges[1]                          = m_vEdges[(lFoundIndex+1)%lTotalEdges];
  SmFilletVertex * pInterpolateEdgeStartV = (SmFilletVertex*)pInterpolateEdge->GetVertex();
  SmFilletVertex * pInterpolateEdgeEndV   = (SmFilletVertex*)pInterpolateEdge->GetOtherVertex(pInterpolateEdgeStartV);
  SmFilletVertex * pOppositeEdgeStartV    = (SmFilletVertex*)pOppositeEdge->GetVertex();
  SmFilletVertex * pOppositeEdgeEndV      = (SmFilletVertex*)pOppositeEdge->GetOtherVertex(pOppositeEdgeStartV);
  SmPoint3d        sOppositeStart         = pOppositeEdgeStartV->GetPoint();
  SmPoint3d        sOppositeEnd           = pOppositeEdgeEndV->GetPoint();

  double           dTol                   = m_dThisApproxTol3d;
  double           dStartRad              = 0.0, dEndRad = 0.0;
  SmFilletSurfaceGeneratorType eType      = SM_FSG_UNKNOWN;
  double           dBlendScale            = 1.0; // For use in SmBlendCurveCrossSectionFSG cases only
  ULONG            lMaxContinuity         = 0;
  SmVector3d       sStartAxis,      sEndAxis;
  SmPoint3d        sStartArcCenter, sEndArcCenter;
  SmBoolean        bApproxArcs            = FALSE;
  double           dApproxArcTol          = 1.0e-3;

  //
  for (ii=0; ii<2; ii++) 
    {
      SmFilletEdge * pE = pCrossEdges[ii];
      if (pE->GetFilletEdgeType() != SM_FE_CROSS_SECTION) 
        { SER(SM_ERR); } // Unknown cases.
        
      SmFilletEdgeuse          * pPrimEU = (SmFilletEdgeuse*)pE->GetPrimaryEdgeuse();
      SmFilletGeom             * pFG     = pPrimEU->GetFilletGeom(); NER(pFG);
      SmFilletSolver           * pFS     = pFG->GetFilletSolver();
      SmFilletSurfaceGenerator * pFSG    = pFS->GetFilletSurfaceGenerator();

      if (ii==0) 
        {
          eType = pFSG->GetFilletSurfaceGeneratorType();
          if (eType == SM_FSG_BLEND_CURVE) 
            {
              //return SM_ERR;  // Right now blend curve one is disabled
              SmBlendCurveCrossSectionFSG * pBlendFSG =
                  (SmBlendCurveCrossSectionFSG*)pFSG;
              dBlendScale = pBlendFSG->GetBlendScale();
              ULONG lContinuity = pBlendFSG->GetContinuity();
              if (lContinuity > lMaxContinuity) 
                { lMaxContinuity = lContinuity; }
            }
        }

      else if (eType != pFSG->GetFilletSurfaceGeneratorType()) 
        { return SM_ERR; } // Unable to convert - two different types
        
      else if (eType == SM_FSG_BLEND_CURVE) 
        {
          SmBlendCurveCrossSectionFSG * pBlendFSG = (SmBlendCurveCrossSectionFSG*)pFSG;
          double dOtherScale = pBlendFSG->GetBlendScale();
          if (smos_Fabs(dBlendScale-dOtherScale) > SM_EFF_ZERO) 
            { return SM_ERR; } // Unable to convert
        }

      // Drop pE's end points to center line to compute radius,
      // axis & origin of cross section
      SmSolution       sSData[8];
      SmSolutionArray  sSolutions(8,sSData);
      SmBSplineCurve * pCenterLine    = pFG->GetCenterLineCurve(); NER(pCenterLine) ;
      SmExtent1d       sCenterLineIvl = pCenterLine->GetNaturalInterval();
      SmVertex       * pV             = pE->GetVertex();
      SmPoint3d        sStartPnt      = pV->GetPoint();
      SmPoint3d        sEndPnt        = pE->GetOtherVertex(pV)->GetPoint();
      SmPoint3d        sArcCenter ;
      double           dRad ;
      SER(pCenterLine->GlobalPointSolve(sCenterLineIvl,
                                        SM_SO_MINIMIZE,
                                        sStartPnt,
                                        dTol,
                                        NULL,
                                        NULL,
                                        SM_SR_SINGLE,
                                        sSolutions));
      if (sSolutions.GetSize() != 1) 
        { SER(SM_ERR); }

      SER(pCenterLine->EvaluatePoint(sSolutions[0].m_vStart[0],sArcCenter));
      SmVector3d sVec = sStartPnt - sArcCenter;
      dRad            = sVec.Length() ;

      SER(pCenterLine->GlobalPointSolve(sCenterLineIvl,
                                        SM_SO_MINIMIZE,
                                        sEndPnt,
                                        dTol,
                                        NULL,
                                        NULL,
                                        SM_SR_SINGLE,
                                        sSolutions));
      if (sSolutions.GetSize() != 1) 
        { SER(SM_ERR); }

      SER(pCenterLine->EvaluatePoint(sSolutions[0].m_vStart[0],sArcCenter));
      SmVector3d sVec1 = sEndPnt - sArcCenter;
      double     dRad1 = sVec1.Length();

      if (smos_Fabs(dRad-dRad1) > dRad*1.0e-8) 
        { SER(SM_ERR); }

      SmVector3d sAxis = sVec * sVec1;
      SER(sAxis.Unitize());

      if (ii==0) { dStartRad = dRad;
                   sStartArcCenter = sArcCenter;
                   sStartAxis = sAxis;
                 }
      else       { dEndRad = dRad;
                   sEndArcCenter = sArcCenter;
                   sEndAxis = sAxis;
                 }
    } // end for each CrossEdge

  // Find base surfaces for filleting
  SmFace          * pFace          = pInterpolateEdge->GetOriginalFace(); NER(pFace);
  SmSurface       * pOrigSurface   = pFace->GetSurface();                 NER(pOrigSurface);

  if (pOppositeEdge->GetFilletEdgeType() != SM_FE_CROSS_SECTION) 
    { return SM_ERR; } // Unable to convert
    
  SmFilletEdgeuse * pOppositePrimEU = (SmFilletEdgeuse*)pOppositeEdge->GetPrimaryEdgeuse();
  SmFilletGeom    * pOppositeFG     = pOppositePrimEU->GetFilletGeom();
  if (pOppositeFG == NULL) 
    { return SM_ERR; } // Unable to convert
    
  SmSurface       * pOppositeFillet = pOppositeFG->GetFilletSurface(); NER(pOppositeFillet);
  SmFilletSolver  * pOppositeFS     = pOppositeFG->GetFilletSolver();

  // Now determine the orientation of pSurface (original surface
  // where pInterpolateEdge is on)
  SmTArray<SmEdge*> sEdges;
  double            dOffsetOrientation = 1.0;
  SmBoolean         bFoundRail = FALSE;
  pInterpolateEdge->GetVertex()->GetEdges(sEdges);

  // 
  for (ii=0; ii<sEdges.GetSize(); ii++) 
    {
      SmFilletEdge * pE = (SmFilletEdge*)sEdges[ii];
      if (pE->GetFilletEdgeType() == SM_FE_RAIL) 
        {
          SmFilletEdgeuse * pEU = (SmFilletEdgeuse*)pE->GetPrimaryEdgeuse();
          SmFilletGeom * pFG = pEU->GetFilletGeom(); NER(pFG);
          SmFilletSolver * pFS = pFG->GetFilletSolver();
          if      (pE == pFG->GetRail(0)) { dOffsetOrientation = pFS->GetOrientation(0); }
          else if (pE == pFG->GetRail(1)) { dOffsetOrientation = pFS->GetOrientation(1); }
          else                            { return SM_ERR; }
          bFoundRail = TRUE;
          break;
        }
    }

  if (!bFoundRail) 
    { return SM_ERR; }

  // Then determine the orientation of pFilletSurface
  double dOffsetOrientation2 = 1.0;

  // But first, drop two end points of pOppositeEdge onto pOppositeFillet
  SmSolution      sSData1[8];
  SmSolutionArray sSolutions1(8,sSData1);
  SmExtent2d      sUVDomain = pOppositeFillet->GetNaturalUVDomain();
  SER(pOppositeFillet->GlobalPointSolve(sUVDomain,
                                        SM_SO_INTERSECT,
                                        sOppositeStart,
                                        dTol,
                                        NULL,
                                        SM_SR_ALL,
                                        sSolutions1));
  if (sSolutions1.GetSize() != 1) 
   { SER(SM_ERR); }

  SmPoint2d sUV = SmPoint2d(sSolutions1[0].m_vStart[0],sSolutions1[0].m_vStart[1]);
  SER(pOppositeFillet->GlobalPointSolve(sUVDomain,
                                        SM_SO_INTERSECT,
                                        sOppositeEnd,
                                        dTol,
                                        NULL,
                                        SM_SR_ALL,
                                        sSolutions1));
  if (sSolutions1.GetSize() != 1) 
    { SER(SM_ERR); }

  SmPoint2d sOtherUV = SmPoint2d(sSolutions1[0].m_vStart[0],sSolutions1[0].m_vStart[1]);

  // Now, derive dOffsetOrientation2
  SmVector3d sSurfNormal;
  SER(pOppositeFillet->EvaluateNormal(sUV,TRUE,TRUE,sSurfNormal));
  sSurfNormal.Unitize();
  SmVector3d sOffsetVec = sEndArcCenter - sOppositeStart;
  if (sSurfNormal.Dot(sOffsetVec) < -SM_EFF_ZERO) 
    { dOffsetOrientation2 = -1.0; }

#ifdef SM_DEBUG_CODE
  if ( DebugLevel() > 0 ) 
    {
      smgfx_SetLook(1,2, 0,0,1); sSurfNormal.Draw(&sOppositeStart); sm_GraphicsLoop();
      smgfx_SetLook(1,2, 0,1,0); sOffsetVec.Draw(&sOppositeStart); sm_GraphicsLoop();
      smgfx_SetLook(1,2, 1,0,0); pOppositeFillet->DrawUV(2,2);
      sOppositeStart.Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // Make a variable radius fillet solver
  // Determine the fillet radius of pOppositeFillet
  SmPoint2d sUV2 = sOtherUV;
  SmPoint2d sSize = sUVDomain.GetSize();

  // note: fillets failed here: but test maybe too demanding (1.0e-12), so modified 9/16/04
  if      (smos_Fabs(smos_Fabs(sUV.y-sOtherUV.y)-sSize.y) < 100*SM_EFF_ZERO) { sUV2.x = sUV.x; } // 100* RCLxx
  else if (smos_Fabs(smos_Fabs(sUV.x-sOtherUV.x)-sSize.x) < 100*SM_EFF_ZERO) { sUV2.y = sUV.y; } // 100* RCLxx
  else                                                                       { SER(SM_ERR); }

  SmPoint3d  sSurfPnt2;
  SmVector3d sSurfNormal2;
  SER(pOppositeFillet->EvaluatePoint(sUV2,sSurfPnt2));
  SER(pOppositeFillet->EvaluateNormal(sUV2,TRUE,TRUE,sSurfNormal2));
  SER(sSurfNormal2.Unitize());

  // Use sOppositeStart, sSurfNormal, sSurfPnt2 & sSurfNormal2
  // to derive radius
  double dOppositeRad;
  double dAngleSpan;
  sm_GetFilletRadius(sOppositeStart,
                     sSurfNormal,
                     sSurfPnt2,
                     sSurfNormal2,
                     dOffsetOrientation2,
                     dOppositeRad,
                     dAngleSpan);

  // Create a temorary center curve
  SmCurve * pOppositeCurve = pOppositeEdge->GetCurve() ; NER(pOppositeCurve);
  SmCurve * pCenterCurve   = NULL;
  SER(pOppositeCurve->Copy(m_crContext,pCenterCurve));
  SmObjDelete sCleanupCC(pCenterCurve); // JLMCC hunting memory leaks

  // Reparametrize the curve with arc-length if the curve is not linear
  //SER(pCenterCurve->ReparametrizeWithArcLength());
  SmExtent1d sIvl = pCenterCurve->GetNaturalInterval();
  // Reverse the curve so it goes along what the fillet goes
  SER(pCenterCurve->ReverseParameterization(sIvl,sIvl));

  // Make sure sStartAxis & sEndAxis follow the curve orientaion
  SER(sStartAxis.Unitize());
  SER(sEndAxis.Unitize());
  SmVector3d sPntVec[2];
  SER(pCenterCurve->Evaluate(sIvl.GetMin(),1,TRUE,sPntVec));
  if (sStartAxis.Dot(sPntVec[1]) < -SM_EFF_ZERO) 
    { sStartAxis = -sStartAxis; }
    
  SER(pCenterCurve->Evaluate(sIvl.GetMax(),1,TRUE,sPntVec));
  if (sEndAxis.Dot(sPntVec[1]) < -SM_EFF_ZERO) 
    { sEndAxis = -sEndAxis; }

#ifdef SM_DEBUG_CODE
  if ( DebugLevel() > 0 ) 
    {
      smgfx_SetLook(1,2, 1,0,0); pOppositeFillet->DrawUV(2,2); sm_GraphicsLoop();
      smgfx_SetLook(1,2, 1,0,0); sStartAxis.Draw(&sStartArcCenter); sm_GraphicsLoop();
      smgfx_SetLook(1,2, 1,0,0); sStartArcCenter.Draw(); sm_GraphicsLoop();
      smgfx_SetLook(1,2, 0,0,1); sEndAxis.Draw(&sEndArcCenter); sm_GraphicsLoop();
      smgfx_SetLook(1,2, 0,0,1); sEndArcCenter.Draw(); sm_GraphicsLoop();
      smgfx_SetLook(1,2, 0,0,1); pCenterCurve->DrawAt(sIvl.GetMin(),1); sm_GraphicsLoop();
      smgfx_SetLook(1,2, 0,0,1); pCenterCurve->Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  SmFilletLaw * pLaw = NULL;
  if (pOppositeFS->m_pFSG->GetFilletSurfaceGeneratorType() != SM_FSG_LINEAR) 
    {
      if (   dOppositeRad <= dEndRad
          || dOppositeRad <= dStartRad) 
        { return (SM_ERR); }

      pLaw = new(m_crContext) SmBSplineFilletLaw(m_crContext,
                                                 dStartRad,
                                                 dEndRad,
                                                 dOppositeRad,
                                                 dAngleSpan,
                                                 0.0,
                                                 0.0);
    }
  else // SM_FSG_LINEAR
    { return SM_ERR; // Not yet handled
      //pLaw = new(m_crContext) SmLinearFilletLaw(dStartRad,dEndRad);
    }
  SmObjDelete sDeleteLaw(pLaw);

  SmVariableRadiusFS * pNewFS = new(m_crContext) SmVariableRadiusFS(m_crContext,
                                                                    dTol,
                                                                    30.0*SM_PI/180.0,
                                                                    2.0*SM_PI/180.0,
                                                                    pOrigSurface,
                                                                    pOppositeFillet,
                                                                    dOffsetOrientation,
                                                                    dOffsetOrientation2,
                                                                    pCenterCurve,
                                                                    *pLaw,
                                                                    FALSE);
  SmObjDelete sDeleteFS(pNewFS);

  pNewFS->SetFilletExecutive(m_pExecutive);
  SmLinearCrossSectionFSG     sFSGLinear;                               // Create Linear
  SmCircularCrossSectionFSG   sFSGCircular(bApproxArcs,dApproxArcTol);  // Creates Circular
  SmBlendCurveCrossSectionFSG sFSGBlend(dBlendScale,lMaxContinuity);    // Create blend

  switch (eType) 
    {   // RCLxx I hope we have etype set by here
      case SM_FSG_CIRCULAR:    pNewFS->SetFilletSurfaceGenerator(&sFSGCircular);
                               break;

      case SM_FSG_LINEAR:      pNewFS->SetFilletSurfaceGenerator(&sFSGLinear);
                               break;

      case SM_FSG_BLEND_CURVE: pNewFS->SetFilletSurfaceGenerator(&sFSGBlend);
                               break;

      case SM_FSG_UNKNOWN:     break;
    }

  SmFilletGeom * pNewFG = pNewFS->GetFirstFilletGeom();

  // Create two rails, one is along the pInterpolateEdge
  SER(SmFilletEdge::MakeFilletEdge(m_pPseudoBrep,           // in : target Brep to receive new topology objects
                                   pInterpolateEdgeStartV,  // in : start of new FilletEdge
                                   pInterpolateEdgeEndV,    // in : end   of new FilletEdge
                                   pNewFG->m_vRails[0]));   // out: newly allocated FilletEdge - no Curve or UVTrimCurve data
                                                            // in : NewFilletEdge's filletCorner, gets stored in rpNewFilletEdge->m_cpCorner
                                                            //      NULL to ignore, default:[NULL]
  // The other one is along the pOppositeFillet but reversed
  SER(SmFilletEdge::MakeFilletEdge(m_pPseudoBrep,           // in : target Brep to receive new topology objects
                                   pOppositeEdgeEndV,       // in : start of new FilletEdge
                                   pOppositeEdgeStartV,     // in : end   of new FilletEdge
                                   pNewFG->m_vRails[1]));   // out: newly allocated FilletEdge - no Curve or UVTrimCurve data
                                                            // in : NewFilletEdge's filletCorner, gets stored in rpNewFilletEdge->m_cpCorner
                                                            //      NULL to ignore, default:[NULL]
  // Prepare for computing the fillet
  SmFilletEdge      * pRail0     = pNewFG->m_vRails[0];
  SmFilletEdge      * pRail1     = pNewFG->m_vRails[1];
  SmFilletEdgeuse   * pEU0       = (SmFilletEdgeuse*)pRail0->GetPrimaryEdgeuse();
  SmFilletEdgeuse   * pEU1       = (SmFilletEdgeuse*)pRail1->GetPrimaryEdgeuse();
  SmFilletVertexuse * pVU0       = (SmFilletVertexuse*)pEU0->GetVertexuse();
  SmFilletVertexuse * pVU1       = (SmFilletVertexuse*)pEU1->GetVertexuse();
  SmTsectPnt        & rTsectPnt0 = pVU0->GetTsectPnt();
  SmTsectPnt        & rTsectPnt1 = pVU1->GetTsectPnt();

  rTsectPnt0.UVPos(0)            = pInterpolateEdgeStartV->GetOriginalUV();
  rTsectPnt0.UVPos(1)            = sOtherUV;
  rTsectPnt1.UVPos(0)            = rTsectPnt0.UVPos(0);
  rTsectPnt1.UVPos(1)            = rTsectPnt0.UVPos(1);
  rTsectPnt0.m_adUserDoubles[0]  = sIvl.GetMin();
  rTsectPnt1.m_adUserDoubles[0]  = sIvl.GetMin();
  rTsectPnt0.m_dCurveParameter   = sIvl.GetMin();
  rTsectPnt1.m_dCurveParameter   = sIvl.GetMin();
  rTsectPnt0.CrvPos()            = sStartArcCenter;
  rTsectPnt0.CrvDeriv()          = sStartAxis;
  rTsectPnt1.CrvPos()            = sStartArcCenter;
  rTsectPnt1.CrvDeriv()          = sStartAxis;
                                   
  pEU0                           = (SmFilletEdgeuse*)pEU0->GetMate();        
  pEU1                           = (SmFilletEdgeuse*)pEU1->GetMate();         
  pVU0                           = (SmFilletVertexuse*)pEU0->GetVertexuse();
  pVU1                           = (SmFilletVertexuse*)pEU1->GetVertexuse();

  SmTsectPnt & rTsectPnt01       = pVU0->GetTsectPnt();
  SmTsectPnt & rTsectPnt11       = pVU1->GetTsectPnt();

  rTsectPnt01.UVPos(0)           = pInterpolateEdgeEndV->GetOriginalUV();
  rTsectPnt01.UVPos(1)           = sUV;
  rTsectPnt11.UVPos(0)           = rTsectPnt01.UVPos(0);
  rTsectPnt11.UVPos(1)           = rTsectPnt01.UVPos(1);

  rTsectPnt01.m_adUserDoubles[0] = sIvl.GetMax();
  rTsectPnt11.m_adUserDoubles[0] = sIvl.GetMax();
  rTsectPnt01.m_dCurveParameter  = sIvl.GetMax();
  rTsectPnt11.m_dCurveParameter  = sIvl.GetMax();

  rTsectPnt01.CrvPos()           = sEndArcCenter;
  rTsectPnt01.CrvDeriv()         = sEndAxis;
  rTsectPnt11.CrvPos()           = sEndArcCenter;
  rTsectPnt11.CrvDeriv()         = sEndAxis;

  pNewFG->SetFilletGeomType(SM_FG_CORNER_FILLET);

  // Compute marching direction for solver
  SER(pCenterCurve->Evaluate(sIvl.GetMin(),1,TRUE,sPntVec));
  if (pNewFS->ReCalcFilletGeom(FALSE,&sPntVec[1],pCenterCurve) != SM_SUCCESS) 
    { return SM_ERR; } // Unable to convert

  if (GetCornerType() == SM_FCR_N_x_N_CONCAVE) 
    {
      // Test if the fillet intersect the corner
      SmCurve * pNewRailRailIntCrv = pRail0->GetCurve();
      SmFilletConcaveNxNCorner * pConcaveNxNCorner = (SmFilletConcaveNxNCorner*)this;
      if (pConcaveNxNCorner->TestRailRailInterpolationCurve(pNewRailRailIntCrv) == TRUE) 
        {
          // This curve intersect eith the corner and we may not use
          // the new fillet as the corner surface
          SM_ASSERT(pRail0 != NULL) ; delete pRail0 ; pRail0 = NULL ;
          SM_ASSERT(pRail1 != NULL) ; delete pRail1 ; pRail1 = NULL ;
          return SM_ERR; // Unable to convert
        }
    }

  // Reset all geometry of corresponding edges
  // Retrieve 3d & uv curves from rails. delete then and
  // replace them by new ones
  SmCurve         * pRail0Curve ;
  SmCurve         * pRail1Curve ;
  SmExtent1d        sCrvIvl;

  // currently:  pRail0 = pNewFG->m_vRails[0];
  //             pRail1 = pNewFG->m_vRails[1];
  //             pEU0   = pRail0->GetPrimaryEdgeuse()->GetMate();
  //             pEU1   = pRail1->GetPrimaryEdgeuse()->GetMate();
  //             pInterpolateEdge = m_vEdges[lFoundIndex];
  //             pOppositeEdge    = m_vEdges[(lFoundIndex+2)%lTotalEdges];
  SmFilletEdgeuse * pInterpolateMateEU          = (SmFilletEdgeuse*)pInterpolateEdge->GetPrimaryEdgeuse()->GetMate() ;

  // save all the UVTrimCurve pointers
  // gwc obsolete:  SmBSplineCurve  * pInterpMateEU_UVTrimCurve   = pInterpolateMateEU->GetUVTrimCurvePointer() ;
  // gwc obsolete:  SmBSplineCurve  * pOppositePrimEU_UVTrimCurve = pOppositePrimEU->GetUVTrimCurvePointer();
  SmBSplineCurve  * pEU0_UVTrimCurve            = pEU0->GetUVTrimCurvePointer() ;
  SmBSplineCurve  * pEU1_UVTrimCurve            = pEU1->GetUVTrimCurvePointer() ;

  // clear references to the UVTrimCurve pointers - to keep them from being deleted at the wrong time
  pInterpolateMateEU->SetUVCurve(NULL, FALSE, FALSE) ; // FALSE = don't delete existing UVCurve, FALSE  no Leak Warns
  pOppositePrimEU->SetUVCurve   (NULL, FALSE, FALSE) ; // FALSE = don't delete existing UVCurve, FALSE  no Leak Warns
  pEU0->SetUVCurve(NULL, FALSE, FALSE) ; // FALSE = don't delete existing UVCurve, FALSE  no Leak Warns
  pEU1->SetUVCurve(NULL, FALSE, FALSE) ; // FALSE = don't delete existing UVCurve, FALSE  no Leak Warns

  // Move Rail0->Curve to pInterpolateEdge->Curve (delete pInterpolateEdge->Curve)
  // gwc obsolete: pCrvToDelete = pInterpolateEdge->GetCurve();
  pRail0Curve  = pRail0->GetCurve(); NER(pRail0Curve);
  sCrvIvl      = pRail0Curve->GetNaturalInterval();
  pRail0->SetCurve(NULL, FALSE, FALSE) ; // FALSE = don't delete preExisting Curve, it's moving, FALSE = no Leak Warns
                                         // side effect: delete current pEdge->UVTrimCurves
  pInterpolateEdge->SetCurve(pRail0Curve, TRUE) ;  // TRUE = delete preExisting Curve
                                                   // side effect: delete current pEdge->UVTrimCurves
  pRail0Curve->SetOwner(pInterpolateEdge);
  pInterpolateEdge->SetInterval(sCrvIvl);
  // gwc obsolete: if (pCrvToDelete) { delete pCrvToDelete; pCrvToDelete = NULL ; }

  // Move Rail1->Curve to pOppositeEdge->Curve (delete pOppositeEdge->Curve) - remember to reverse Curve for Opposite Edge
  // gwc obsolete: pCrvToDelete = pOppositeEdge->GetCurve();
  pRail1Curve  = pRail1->GetCurve(); NER(pRail1Curve);
  pRail1->SetCurve(NULL, FALSE, FALSE) ; // FALSE = don't delete preExisting Curve, it's moving, FALSE = no Leak Warns
                                  // side effect: delete current pEdge->UVTrimCurves
  // Similarily, reverse 3d curve
  sCrvIvl = pRail1Curve->GetNaturalInterval();
  SER(pRail1Curve->ReverseParameterization(sCrvIvl,sCrvIvl));
  pOppositeEdge->SetCurve(pRail1Curve, TRUE) ;  // TRUE = delete preExisting Curve
                                                // side effect: delete current pEdge->UVTrimCurves
  pRail1Curve->SetOwner(pOppositeEdge);
  pOppositeEdge->SetInterval(sCrvIvl);
  // gwc obsolete: if (pCrvToDelete) { delete pCrvToDelete; pCrvToDelete = NULL ; }

  // Begin scope to move pEU0 and pEU1 UVTrimCurves to InterpolateMateEU and OppositePrimEU
    {
      // gwc obsolete: // free InterpolatedMateEU->UVTrimCurve
      // gwc obsolete: if (pInterpMateEU_UVTrimCurve) { delete pInterpMateEU_UVTrimCurve; pInterpMateEU_UVTrimCurve = NULL ; }

      // move pEU0->UVTrimCurve to pInterpolatedMateEU
      pEU0->SetUVCurve(NULL, FALSE, FALSE) ; // FALSE = don't delete existing UVCurve, FALSE  no Leak Warns
      pInterpolateMateEU->SetUVCurve(pEU0_UVTrimCurve, TRUE) ;  // TRUE = delete preExisting UVCurve

      // gwc obsolete: // free pOppositePrimEU->UVTrimCurve
      // gwc obsolete: if (pOppositePrimEU_UVTrimCurve) { delete pOppositePrimEU_UVTrimCurve; pOppositePrimEU_UVTrimCurve = NULL ; }

      // move pEU1->UVTrimCurve to pOppositePrimEU - remember to reverse UVTrimCurve for Opposite EU
      SmExtent1d sUVCrvIvl = pEU1_UVTrimCurve->GetNaturalInterval();
      SER(pEU1_UVTrimCurve->ReverseParameterization(sUVCrvIvl,sUVCrvIvl));
      pEU1->SetUVCurve(NULL, FALSE, FALSE) ; // FALSE = don't delete existing UVCurve, FALSE  no Leak Warns
      pOppositePrimEU->SetUVCurve(pEU1_UVTrimCurve, TRUE) ;  // TRUE = delete preExisting UVCurve
    } // end scope to move pEU0 and pEU1 UVTrimCurves to InterpolateMateEU and OppositePrimEU

  // set output
  rpSurface = pNewFG->GetFilletSurface();

  // Now, the last step is to update adjacent fillet geoms
  //SmFace * pOppositeFace = (SmFace*)pOppositeFillet->GetOwner();
  //NER(pOppositeFace);
  //pOppositeFillet->SetOwner(NULL);
  //SmBrep * pBrep = pOppositeFace->GetBrep();
  //SmTArray<SmFace*> sFaces;
  //sFaces.Add(pOppositeFace);
  //pBrep->RemoveFaces(sFaces);  // increments unlocked mark value

  // Remake adjacent fillet faces
  //SER(pOppositeFG->MakeFaceBrep(pBrep));

  // all done
  return SM_SUCCESS;

} // end SmFilletCorner::MakeVariableRadiusFillet

/*******************************************************************//**
PURPOSE: In the case where we have a 3-sided corner (3x3), try to
    convert it to a constant-radius fillet to achive better G1 continuity.

NOTES: Return SM_SUCCESS if successfully create a fillet surface
    for this corner. Otherwise, return SM_ERR.
***********************************************************************/
SmStatus SmFilletCorner::MakeConstantRadiusFillet
 // gwc: I think pSideSurface no longer has to be a BBSplineSurface - replace 1 line
 // rm : (SmBSplineSurface *& rpSurface)
 (SM_FILLETSURF_TYPE *& rpSurface)
{
  // check state
  ULONG lTotalEdges = m_vEdges.GetSize();
  if (lTotalEdges != 3) 
    { return SM_ERR; } // Unable to convert

  // Find surfaces of blend

  // Find the shortest Edge.
  ULONG ii, lFoundIndex = 9999;
  double dMinDist = SM_BIG_DOUBLE;
  for (ii=0; ii<m_vEdges.GetSize(); ii++) 
    {
      SmFilletEdge * pE = m_vEdges[ii];
      if (pE->GetFilletEdgeType() != SM_FE_CROSS_SECTION) 
        { return SM_ERR; } // Unable to convert

      SmVertex * pV      = pE->GetVertex();
      SmVertex * pOtherV = pE->GetOtherVertex(pV);
      double dLeng = pV->GetPoint().DistanceBetween(pOtherV->GetPoint());
      if ( dLeng < dMinDist ) 
        {
          dMinDist = dLeng;
          lFoundIndex = ii;
        }
    }

  SmFilletEdge   * pBaseE   = m_vEdges[lFoundIndex];  // Shortest Edge
  SmAxis2Placement sRefFrame;
  double           dStartAng, dEndAng;
  double           dFilletRad;
  double           dTol     = m_dThisApproxTol3d;
  SmBSplineCurve * pBaseArc = SM_CAST_PTR(SmBSplineCurve,pBaseE->GetCurve());
  // Determine the radius of fillet
  if (!pBaseArc->IsArc(5,dTol,sRefFrame,dFilletRad,dStartAng,dEndAng)) 
     { return SM_ERR; } // Unable to convert
  

  SmFilletEdge    * pEdge0    = m_vEdges[((lFoundIndex+lTotalEdges)-1)%lTotalEdges];
  SmFilletEdge    * pEdge1    = m_vEdges[(lFoundIndex+1)%lTotalEdges];
  SmFilletEdgeuse * pPrimEU0  = (SmFilletEdgeuse*)pEdge0->GetPrimaryEdgeuse();
  SmFilletGeom    * pFG0      = pPrimEU0->GetFilletGeom(); NER(pFG0);
  SmSurface       * pSurface0 = pFG0->GetFilletSurface();  NER(pSurface0);
  SmFilletEdgeuse * pPrimEU1  = (SmFilletEdgeuse*)pEdge1->GetPrimaryEdgeuse();
  SmFilletGeom    * pFG1      = pPrimEU1->GetFilletGeom(); NER(pFG1);
  SmSurface       * pSurface1 = pFG1->GetFilletSurface();  NER(pSurface1);

  // Create a temorary center curve
  const SmPoint3d & rLineEnd     = sRefFrame.GetOriginRef();
  SmVertex        * pOppositeV   = pEdge0->GetVertex();
  SmPoint3d         sOppositePnt = pOppositeV->GetPoint();
  SmVector3d        sZAxis       = sRefFrame.GetZAxis();
  double dT;

  SER(smgu_LineClosestPoint(rLineEnd,sZAxis,sOppositePnt,dT));
  SmPoint3d sLineStart   = rLineEnd + sZAxis*dT;
  SmLine  * pCenterCurve = NULL;
  SER(SmLine::CreateLineSegment(m_crContext,3,sLineStart,rLineEnd,pCenterCurve));
  SmObjDelete sDeleteLine(pCenterCurve);

  // Drop both ends of base curve onto two fillets
  SmVertex      * pV         = pBaseE->GetVertex();
  SmPoint3d       sPnt       = pV->GetPoint();
  SmSolutionArray sSolutions;
  SmExtent2d      sUVDomain0 = pSurface0->GetNaturalUVDomain();

  SER(pSurface0->GlobalPointSolve(sUVDomain0,
                                  SM_SO_INTERSECT,
                                  sPnt,
                                  dTol,
                                  NULL,
                                  SM_SR_ALL,
                                  sSolutions));

  if (sSolutions.GetSize() != 1) SER(SM_ERR);
  SmPoint2d sUV = SmPoint2d(sSolutions[0].m_vStart[0],sSolutions[0].m_vStart[1]);

  SmVertex * pOtherV = pBaseE->GetOtherVertex(pV);
  SmPoint3d  sOtherPnt = pOtherV->GetPoint();
  SmExtent2d sUVDomain1 = pSurface1->GetNaturalUVDomain();
  SER(pSurface1->GlobalPointSolve(sUVDomain1,
                                  SM_SO_INTERSECT,
                                  sOtherPnt,
                                  dTol,
                                  NULL,
                                  SM_SR_ALL,
                                  sSolutions));

  if (sSolutions.GetSize() != 1) SER(SM_ERR);
  SmPoint2d sOtherUV = SmPoint2d(sSolutions[0].m_vStart[0],sSolutions[0].m_vStart[1]);

#ifdef SM_DEBUG_CODE
  if ( DebugLevel() > 0 ) 
    {
      sm_GraphicsLoop();
      smgfx_SetLook(1,2, 1,0,0); pSurface0->DrawUV(2,2); sm_GraphicsLoop();
      pSurface1->DrawUV(2,2); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  SmFilletSurfaceGenerator   * pFSG0         = pFG0->GetFilletSolver()->GetFilletSurfaceGenerator();
  SmFilletSurfaceGenerator   * pFSG1         = pFG1->GetFilletSolver()->GetFilletSurfaceGenerator();
  SmFilletSurfaceGeneratorType eType         = pFSG0->GetFilletSurfaceGeneratorType();
  double                      dBlendScale    = 1.5; // For use in SmBlendCurveCrossSectionFSG cases only
  ULONG                       lMaxContinuity = 0;
  if (eType != pFSG1->GetFilletSurfaceGeneratorType()) 
    { return SM_ERR ; }

  if (eType == SM_FSG_BLEND_CURVE) 
    {
      SmBlendCurveCrossSectionFSG * pBlendFSG = (SmBlendCurveCrossSectionFSG*)pFSG0;
      dBlendScale                             = pBlendFSG->GetBlendScale();
      pBlendFSG                               = (SmBlendCurveCrossSectionFSG*)pFSG1;

      if (smos_Fabs(dBlendScale-pBlendFSG->GetBlendScale()) > SM_EFF_ZERO) 
        { return SM_ERR ; } // Unable to convert
     
      ULONG lContinuity = pBlendFSG->GetContinuity();

      if (lContinuity > lMaxContinuity) 
        { lMaxContinuity = lContinuity; }
    }

  // Make a constant radius fillet solver
  SmFilletSolver            * pNewFS = NULL;
  SmFilletLaw               * pLaw   = NULL;
  SmLinearCrossSectionFSG     sFSGLinear;                            // Create Linear
  SmCircularCrossSectionFSG   sFSGCircular(TRUE,1.0e-3);             // Creates Circular
  SmBlendCurveCrossSectionFSG sFSGBlend(dBlendScale,lMaxContinuity); // Create blend
  switch (eType) 
    {
      case SM_FSG_CIRCULAR:
          pNewFS = new(m_crContext) SmConstantRadiusFS(m_crContext,
                                                       dTol*0.1,
                                                       10.0*SM_PI/180.0,
                                                       2.0*SM_PI/180.0,
                                                       dFilletRad,
                                                       dFilletRad,
                                                       *pSurface0,
                                                       *pSurface1,
                                                       TRUE,
                                                       TRUE);
          pNewFS->SetFilletSurfaceGenerator(&sFSGCircular);
          break;
      case SM_FSG_BLEND_CURVE:
          pNewFS = new(m_crContext) SmConstantRadiusFS(m_crContext,
                                                       dTol*0.1,
                                                       10.0*SM_PI/180.0,
                                                       2.0*SM_PI/180.0,
                                                       dFilletRad,
                                                       dFilletRad,
                                                       *pSurface0,
                                                       *pSurface1,
                                                       TRUE,
                                                       TRUE);
          pNewFS->SetFilletSurfaceGenerator(&sFSGBlend);
          break;
      case SM_FSG_LINEAR:
          return SM_ERR;
#ifdef FIX_LINEAR_CASE
          {
          // Now determine the orientation of pSurface0
          double dOffsetOrientation0 = 1.0;
          SmVector3d sSurfNormal0;
          SER(pSurface0->EvaluateNormal(sUV,TRUE,TRUE,sSurfNormal0));
          sSurfNormal0.Unitize();
          SmVector3d sOffsetVec0 = sLineStart - sPnt;
          if (sSurfNormal0.Dot(sOffsetVec0) < -SM_EFF_ZERO) {
              dOffsetOrientation0 = -1.0;
          }
          // Now determine the orientation of pSurface1
          double dOffsetOrientation1 = 1.0;
          SmVector3d sSurfNormal1;
          SER(pSurface1->EvaluateNormal(sOtherUV,TRUE,TRUE,sSurfNormal1));
          sSurfNormal1.Unitize();
          SmVector3d sOffsetVec1 = sLineStart - sOtherPnt;
          if (sSurfNormal1.Dot(sOffsetVec1) < -SM_EFF_ZERO) {
              dOffsetOrientation1 = -1.0;
          }
          pLaw = new(m_crContext) SmLinearFilletLaw(0.0,dFilletRad);
          pNewFS = new(m_crContext) SmVariableRadiusFS(m_crContext,
              dTol,30.0*SM_PI/180.0,2.0*SM_PI/180.0,pSurface0,pSurface1,
              dOffsetOrientation0,dOffsetOrientation1,pCenterCurve,*pLaw,FALSE);
          pNewFS->SetFilletSurfaceGenerator(&sFSGLinear);
          }
#endif // SM_DEBUG_CODE
          break;
      case SM_FSG_UNKNOWN:
          break;
  }
  SmObjDelete sDeleteLaw; 
  if (pLaw) { sDeleteLaw.SetObj(pLaw); }

  SmObjDelete sDeleteFS(pNewFS);
  pNewFS->SetFilletExecutive(m_pExecutive);
  SmFilletGeom * pNewFG = pNewFS->GetFirstFilletGeom();

  // Create two rails, one is along the pInterpolateEdge
  SER(SmFilletEdge::MakeFilletEdge(m_pPseudoBrep,                // in : target Brep to receive new topology objects
                                   (SmFilletVertex*)pOppositeV,  // in : start of new FilletEdge
                                   (SmFilletVertex*)pV,          // in : end   of new FilletEdge
                                   pNewFG->m_vRails[0]));        // out: newly allocated FilletEdge - no Curve or UVTrimCurve data
                                                                 // in : NewFilletEdge's filletCorner, gets stored in rpNewFilletEdge->m_cpCorner
                                                                 //      NULL to ignore, default:[NULL]
  // The other one is along the pOppositeFillet but reversed
  SER(SmFilletEdge::MakeFilletEdge(m_pPseudoBrep,                // in : target Brep to receive new topology objects
                                   (SmFilletVertex*)pOppositeV,  // in : start of new FilletEdge
                                   (SmFilletVertex*)pOtherV,     // in : end   of new FilletEdge
                                   pNewFG->m_vRails[1]));        // out: newly allocated FilletEdge - no Curve or UVTrimCurve data
                                                                 // in : NewFilletEdge's filletCorner, gets stored in rpNewFilletEdge->m_cpCorner
                                                                 //      NULL to ignore, default:[NULL]
  // Prepare for computing the fillet
  SmFilletEdge      * pRail0     = pNewFG->m_vRails[0];
  SmFilletEdge      * pRail1     = pNewFG->m_vRails[1];
  SmFilletEdgeuse   * pEU0       = (SmFilletEdgeuse*)pRail0->GetPrimaryEdgeuse();
  SmFilletEdgeuse   * pEU1       = (SmFilletEdgeuse*)pRail1->GetPrimaryEdgeuse();
  SmFilletVertexuse * pVU0       = (SmFilletVertexuse*)pEU0->GetVertexuse();
  SmFilletVertexuse * pVU1       = (SmFilletVertexuse*)pEU1->GetVertexuse();
  SmTsectPnt        & rTsectPnt0 = pVU0->GetTsectPnt();
  SmTsectPnt        & rTsectPnt1 = pVU1->GetTsectPnt();

  // Drop sOppositePnt onto both surfaces
  SER(pSurface0->GlobalPointSolve(sUVDomain0,SM_SO_INTERSECT, sOppositePnt,dTol,NULL,SM_SR_ALL,sSolutions));
  if (sSolutions.GetSize() != 1)
    { SER(SM_ERR); }
  SmPoint2d sOppositeUV = SmPoint2d(sSolutions[0].m_vStart[0],sSolutions[0].m_vStart[1]);

  SER(pSurface1->GlobalPointSolve(sUVDomain1,SM_SO_INTERSECT, sOppositePnt,dTol,NULL,SM_SR_ALL,sSolutions));
  if (sSolutions.GetSize() != 1)
    { SER(SM_ERR); }
  SmPoint2d sOppositeOtherUV = SmPoint2d(sSolutions[0].m_vStart[0],sSolutions[0].m_vStart[1]);

  rTsectPnt0.UVPos(0)      = sOppositeUV;
  rTsectPnt0.UVPos(1)      = sOppositeOtherUV;
  rTsectPnt1.UVPos(0)      = rTsectPnt0.UVPos(0);
  rTsectPnt1.UVPos(1)      = rTsectPnt0.UVPos(1);

  pEU0                     = (SmFilletEdgeuse*)pEU0->GetMate();
  pEU1                     = (SmFilletEdgeuse*)pEU1->GetMate();
  pVU0                     = (SmFilletVertexuse*)pEU0->GetVertexuse();
  pVU1                     = (SmFilletVertexuse*)pEU1->GetVertexuse();

  SmTsectPnt & rTsectPnt01 = pVU0->GetTsectPnt();
  SmTsectPnt & rTsectPnt11 = pVU1->GetTsectPnt();

  rTsectPnt01.UVPos(0)     = sUV;
  rTsectPnt01.UVPos(1)     = sOtherUV;
  rTsectPnt11.UVPos(0)     = rTsectPnt01.UVPos(0);
  rTsectPnt11.UVPos(1)     = rTsectPnt01.UVPos(1);

  pNewFG->SetFilletGeomType(SM_FG_CORNER_FILLET);
  // Compute marching direction for solver
  SmPoint3d sMidPnt;
  SER(pBaseArc->EvaluatePoint(pBaseArc->GetNaturalInterval().Evaluate(0.5),sMidPnt));
  SmVector3d sMarchDir = sMidPnt - sOppositePnt;
  if (pNewFG->ReCalcFilletGeom(FALSE,&sMarchDir,pCenterCurve) != SM_SUCCESS) 
    { return SM_ERR ; } // Unable to convert

  // EdgeUse current UVTrimCurves
  SmCurve        * pRail0Curve          = pRail0->GetCurve(); NER(pRail0Curve);
  SmCurve        * pRail1Curve          = pRail1->GetCurve(); NER(pRail1Curve);
  //SmBSplineCurve * pPrimEU0_UVTrimCurve = pPrimEU0->GetUVTrimCurvePointer() ;
  //SmBSplineCurve * pPrimEU1_UVTrimCurve = pPrimEU1->GetUVTrimCurvePointer();
  SmBSplineCurve * pEU0_UVTrimCurve     = pEU0->GetUVTrimCurvePointer();
  SmBSplineCurve * pEU1_UVTrimCurve     = pEU1->GetUVTrimCurvePointer();

  // Clear EU UVTrimCurve ptrs to prevent deleting them at the wrong time
  pPrimEU0->SetUVCurve(NULL, FALSE, FALSE) ; // FALSE = don't delete existing UVTrimCurve, FALSE = don't warn
  pPrimEU1->SetUVCurve(NULL, FALSE, FALSE) ; // FALSE = don't delete existing UVTrimCurve, FALSE = don't warn
  pEU0->SetUVCurve    (NULL, FALSE, FALSE) ; // FALSE = don't delete existing UVTrimCurve, FALSE = don't warn
  pEU1->SetUVCurve    (NULL, FALSE, FALSE) ; // FALSE = don't delete existing UVTrimCurve, FALSE = don't warn

  // Reset all geometry of corresponding edges
  // Retrieve 3d & uv curves from rails. delete then and
  // replace them by new ones

  // move pRail0->Curve to pEdge0 (delete current pEdge0->Curve)
  SmExtent1d sCrvIvl;

  // gwc obsolete: pCrvToDelete = pEdge0->GetCurve();
  pRail0->SetCurve(NULL, FALSE, FALSE) ; // FALSE = don't delete preExisting Curve - it's moving, FALSE = no leak warnings
                                         // side effect: delete current pEdge->UVTrimCurves
  sCrvIvl = pRail0Curve->GetNaturalInterval();
  pEdge0->SetCurve(pRail0Curve, TRUE) ;  // TRUE = delete preExisting Curve
                                         // side effect: delete current pEdge->UVTrimCurves
  pRail0Curve->SetOwner(pEdge0);
  pEdge0->SetInterval(sCrvIvl);
  // gwc obsolete: if (pCrvToDelete) { delete pCrvToDelete; pCrvToDelete = NULL ; }
    
  // move pRail1->Curve to pEdge1 (delete current pEdge1->Curve - reverse Rail1Curve direction)
  // gwc obsolete: pCrvToDelete = pEdge1->GetCurve();
  pRail1->SetCurve(NULL, FALSE, FALSE) ; // FALSE = don't delete preExisting Curve - it's moving, FALSE = no leak warnings
                                         // side effect: delete current pEdge->UVTrimCurves
  sCrvIvl = pRail1Curve->GetNaturalInterval();
  SER(pRail1Curve->ReverseParameterization(sCrvIvl,sCrvIvl));
  pEdge1->SetCurve(pRail1Curve, TRUE) ;  // TRUE = don't delete preExisting Curve
                                         // side effect: delete current pEdge->UVTrimCurves
  pRail1Curve->SetOwner(pEdge1);
  pEdge1->SetInterval(sCrvIvl);
  // gwc obsolete: if (pCrvToDelete) { delete pCrvToDelete; pCrvToDelete = NULL ; }
    
  // move pEU0->UVTrimCurve to pPrimEU0 (delete current pPrimEU0->UVTrimCurve)
  // gwc obsolete: if (pPrimEU0_UVTrimCurve) { delete pPrimEU0_UVTrimCurve; pPrimEU0_UVTrimCurve = NULL ; }
  pPrimEU0->SetUVCurve(pEU0_UVTrimCurve, TRUE); // TRUE = delete any existing UVTrimCurve

  // move pEU1->UVTrimCurve to pPrimEU1 (delete current pPrimEU1->UVTrimCurve - reverse UVTrimCurve direction)
  // gwc obsolete: if (pPrimEU1_UVTrimCurve) { delete pPrimEU1_UVTrimCurve; pPrimEU1_UVTrimCurve = NULL ; }
  if( pEU1_UVTrimCurve )    { SmExtent1d sUVCrvIvl = pEU1_UVTrimCurve->GetNaturalInterval();
                              SER(pEU1_UVTrimCurve->ReverseParameterization(sUVCrvIvl,sUVCrvIvl));
                            }
  pPrimEU1->SetUVCurve(pEU1_UVTrimCurve, TRUE); // TRUE = delete any existing UVTrimCurve

  rpSurface = pNewFG->GetFilletSurface();

  // Now, the last step is to update adjacent fillet geoms
  //SmFace * pFace0 = (SmFace*)pSurface0->GetOwner();
  //SmFace * pFace1 = (SmFace*)pSurface1->GetOwner();
  //NER(pFace0); NER(pFace1);
  //pSurface0->SetOwner(NULL);
  //pSurface1->SetOwner(NULL);
  //SmBrep * pBrep = pFace0->GetBrep();
  //SmTArray<SmFace*> sFaces;
  //sFaces.Add(pFace0);
  //sFaces.Add(pFace1);
  //pBrep->RemoveFaces(sFaces);  // increments unlocked mark value
  // Remake adjacent fillet faces
  //SER(pFG0->MakeFaceBrep(pBrep));
  //SER(pFG1->MakeFaceBrep(pBrep));

  return SM_SUCCESS;

} // end SmFilletCorner::MakeConstantRadiusFillet

/*******************************************************************//**
PURPOSE: if an AnalyticSurface can approx FilletCorner->Curves to tol, 
           Make and store that AnalyticSurface in m_vSurfaces, and return TRUE
         else return FALSE. 

NOTES: 
***********************************************************************/
SmBoolean SmFilletCorner::MakeAnalyticCornerSurface()
{
  // no work - Vertex does not have exactly 3 or 4 edges
  ULONG lTotalEdges = m_vEdges.GetSize();
  if (lTotalEdges < 3 || lTotalEdges > 4) 
    { return FALSE; }

  // no work - quit if FilletVertex is connected to any BlendCurve FilletEdge
  ULONG ii;
  for (ii=0; ii<m_vEdges.GetSize(); ii++)
    {
      SmFilletEdge    * pFilletEdge = m_vEdges[ii];
      SmFilletEdgeuse * pPrimEU     = (SmFilletEdgeuse*)pFilletEdge->GetPrimaryEdgeuse();
      SmFilletGeom    * pFilletGeom = pPrimEU->GetFilletGeom();

      if (pFilletGeom)
        {
          SmFilletSolver           * pFS  = pFilletGeom->GetFilletSolver();
          SmFilletSurfaceGenerator * pFSG = pFS->GetFilletSurfaceGenerator();
          if (pFSG->GetFilletSurfaceGeneratorType() == SM_FSG_BLEND_CURVE)
            {
              // corner is not analytic when connected to a BlendCurve
              return FALSE;
            }
        }
    } // end iter every FilletEdge looking for BlendCurve Fillets

  // locals
  SmBoolean                 bCreateSurfOfRevo = FALSE;
  SmTArray<SmBSplineCurve*> sCurves;
  SmTArray<SmPoint3d>       sPlanePnts;
  ULONG                     lMajorArcIndex = 0;

  // for every FilletEdge connected to this FilletVertex 
  //  - check FilletEdge types and quit for those that can't connect to an analytic corner
  //  - otherwise branch on FilletEdge Type to gather - bCreateSurfOfRevo: TRUE = build SurfOfRevoltion corner
  //                                                  - lMajorArcIndex   : by sweeping this FilletEdge's endCurve
  //                                                  - sPlanePnts       : list of all connected FilletEdge->StartPts
  //                                                  - sCurves          : list of all connected FilletEdge->Curves
  for (ii=0; ii<m_vEdges.GetSize(); ii++) 
    {
      SmFilletEdge * pFilletEdge = m_vEdges[ii];

      // branch on FilletCorner type to capture information about corner surface to be built
      if (m_bChamfer) 
        { 
          // corner will be planar - depends on FilletEdges being given in CCW order about Vertex 
          //                       - order sets the surface normal direction
          sPlanePnts.Add(pFilletEdge->GetStartVertex()->GetPoint()); 
        } // end checking for Chamfer (planar) attached FilletEdges branch

      else if (pFilletEdge->GetFilletEdgeType() == SM_FE_RAIL_RAIL_INTERPOLATION) 
        {
          // can't make analytics for CUBIC_RAIL_RAIL_INTERPOLATE case
          if (pFilletEdge->GetStatus() == SM_FE_CUBIC_RAIL_RAIL_INTERPOLATE) 
            {
              return FALSE; // Unable to create analytic
            }
          // otherwise - let the last FilletEdge seen by the Major Arc for a SurfOfRevolution
          bCreateSurfOfRevo = TRUE;
          lMajorArcIndex = ii;
        } // end checking for RailRailInterpolation attached FilletEdges branch

      else // quit if any FilletEdge is a VariableRadius Fillet
        {
          // Get pFilletEdge->FilletGeom
          SmFilletEdgeuse * pPrimEU     = (SmFilletEdgeuse*)pFilletEdge->GetPrimaryEdgeuse();
          SmFilletGeom    * pFilletGeom = pPrimEU->GetFilletGeom();

          // if FilletGeom is not on PrimEU look at PrimEU->Mate
          if (!pFilletGeom) { pPrimEU = (SmFilletEdgeuse*)pPrimEU->GetMate();
                              pFilletGeom = pPrimEU->GetFilletGeom();
                            }

          // no work - VariableRadius Fillet Edges can't connect to analytic FilletCorners
          if (pFilletGeom) { SmFilletSolver * pFS = pFilletGeom->GetFilletSolver();
                             if (pFS->GetSolverType() == SM_FS_VARIABLE_RADIUS) 
                               {
                                 return FALSE;
                               }
                           }
        } // end checking for attached variableRadius FilletEdges branch

      // gather all connected FilletEdge->Curves - quit if any edge is not a BSplineCurve
      // gwc: is the BSPlineCurve restriction relevant?
      SmBSplineCurve *pBSC = SM_CAST_PTR(SmBSplineCurve, m_vEdges[ii]->GetCurve());
      if (!pBSC) SER(SM_ERR); // Not Nurbs??
      sCurves.Add(pBSC);

    } // end iter every attached FilletEdge - branching on type and collecting FilletEdge->Curves

  //if (!bCreateSurfOfRevo && lTotalEdges == 3) 
  //  {
  //    double dMinRad = SM_BIG_DOUBLE;
  //    for (ULONG jj=0; jj<m_vEdges.GetSize(); jj++) 
  //      {
  //        SmBSplineCurve *pBSC = sCurves[jj];
  //        SmAxis2Placement sRefFrame;
  //        double dRad, dStartAng, dEndAng;
  //        if (!pBSC->IsArc(5,d3DTolerance,sRefFrame,dRad,dStartAng,dEndAng)) 
  //          {
  //            break;
  //          }
  //        if (dRad < dMinRad) 
  //          {
  //            lMajorArcIndex = jj;
  //            dMinRad = dRad;
  //          }
  //      }
  //  }

  // local Ptr to FilletSurface to be made for this FilletCorner
  // gwc: I think pSideSurface no longer has to be a BBSplineSurface - replace 1 line
  // rm : SmBSplineSurface *pSurface = NULL;
  SM_FILLETSURF_TYPE *pSurface = NULL;

  int iDebugLevel = 0;
#ifdef SM_DEBUG_CODE
  iDebugLevel = DebugLevel();
#endif // SM_DEBUG_CODE

  // For FilletCorners connected to FilletEdge chamfers: pSurface =   3FilletEdges ? SmPlane 
  //                                                                : 4FilletEdges ? Bilinear SmBSplineSurface
  if (m_bChamfer) 
    {
      // check assumption: if one FilletEdge is a chamfer - all FilletEdges are Chamfers
      SM_ASSERT(sPlanePnts.GetSize() == lTotalEdges);

      // Create a 'planar' surface - SmPlane for 3pts, SmBSpline BilinearSurface for 4 pts
      if(SM_SUCCESS != sm_CreatePlanarSurface(m_crContext,           // in : context for new object construction
                                              sPlanePnts,            // in : ControlPt List ordered:3Pts = any CCW sequence - normal set for right hand rule
                                                                     //                             4Pts = [Pw[0][0], Pw[1][0], Pw[1][1], Pw[0][1]]
                                              m_dThisApproxTol3d,    // in : max allowed input point to output plane deviation
                                              pSurface))             // out: for 3 pts - pt interpolating SmPlane centered on PtCentroid,
                                                                     //          4 pts - Bilinear BSplineSurface using pts as corners,
                                                                     //          else  - signal "Not Yet Implemented" error
        { // quit when Vertex is not connected to exactly 3 or 4 FilletEdges
          return FALSE; 
        }

      // Re-compute geometry of each edge
      for (ii=0; ii<m_vEdges.GetSize(); ii++) 
        {
          SmFilletEdge    * pFilletEdge = m_vEdges[ii];
          SmFilletEdgeuse * pPrimEU     = (SmFilletEdgeuse*)pFilletEdge->GetPrimaryEdgeuse();
          SmBSplineCurve  * pBSC        = SM_CAST_PTR(SmBSplineCurve,pFilletEdge->GetCurve());
          SmPoint3d         sLinePt;
          SmVector3d        sLineVec;

          // No work - if any FilletEdge is not a BSplineCurve - gwc: why?
          //         - never come down this path because this condition is already checked
          //           otherwise it's a memory leak for pSurface
          if (!pBSC) 
            { SER(SM_ERR) ; }

          // skip FilletEdges which are linear
          if (pBSC->IsLine(10, m_dThisApproxTol3d, sLinePt, sLineVec)) 
            {
              continue; // No need to recompute the edge
            }

          // Delete FilletEdge->Curve and UVTrimCurve
          pFilletEdge->SetCurve(NULL, TRUE) ; // TRUE = delete preExisting Curve
                                              // side effect: delete current pEdge->UVTrimCurves
          pBSC = NULL ; 

          //  // gwc: this is a bug - leaves stale pointers on pFilletEdge - use SetCurve() instead
          //  SM_ASSERT(pBSC != NULL) ; delete pBSC ; pBSC = NULL ;
          //  SmBSplineCurve * pOldUVCurve1 = pPrimEU->GetUVTrimCurvePointer();
          //  if (pOldUVCurve1) { delete pOldUVCurve1; pOldUVCurve1 = NULL ; }
          
          // Intersect FilletCorner->plane with FilletEdge->Surface


          double dApproxTol, dAngTol;
          // gwc: I think pSideSurface no longer has to be a BBSplineSurface - replace 1 line
          // rm : SmBSplineSurface * pFilletSurface = NULL;
          SM_FILLETSURF_TYPE * pFilletSurface = NULL;
          if (pFilletEdge->GetFilletEdgeType() == SM_FE_RAIL_RAIL_INTERPOLATION) 
            {
              // Get original face surface
              SmFace * pOrigFace = pFilletEdge->GetOriginalFace(); NER(pOrigFace);
              // gwc: I think pSideSurface no longer has to be a BBSplineSurface - replace 1 line
              // rm : pFilletSurface     = SM_CAST_PTR(SmBSplineSurface,pOrigFace->GetSurface());
              pFilletSurface     = SM_CAST_FILLETSURF_PTR(SM_FILLETSURF_TYPE, pOrigFace->GetSurface()) ;
              dApproxTol         = pOrigFace->GetTolerance();
              dAngTol            = 30.0*SM_PI/180.0;
            }
          else 
            {
              SmFilletGeom   * pFilletGeom   = pPrimEU->GetFilletGeom();
              SmFilletSolver * pFilletSolver = pFilletGeom->GetFilletSolver();
              pFilletSurface                 = pFilletGeom->GetFilletSurface();
              dApproxTol                     = pFilletSolver->GetThisApproxTol3d();
              dAngTol                        = pFilletSolver->GetThisAngTolRad();
            }
          NER(pFilletSurface);

          // locals
          SmBSplineCurve * pNew3DCurve = NULL;
          SmBSplineCurve * pUVCurve1   = NULL;
          SmBSplineCurve * pUVCurve2   = NULL;
          SmVertex       * pStartV     = pFilletEdge->GetStartVertex();
          SmVertex       * pEndV       = pFilletEdge->GetOtherVertex(pStartV);
          SmVector3d       sDir        = pEndV->GetPoint() - pStartV->GetPoint();

          // intersed 
          SER(sm_SrfSrfIntersection( m_crContext,    // in : context for new object construction
                                     pStartV,        // in : 1st point known to be on intersection curve
                                     pEndV,          // in : 2nd point known to be on intersection curve
                                    &sDir,           // in : expected general direction of intersection curve from 1st point
                                     NULL,           // in   expected intersection end direction
                                     pFilletSurface, // in : 1st intersecting surface
                                     pSurface,       // in : 2nd intersecting surface
                                     dApproxTol,     // in : max allowed distance between xSect Curve and surfaces
                                     dAngTol,        // in : max allowed angle between consecutive xSect curve segment tangents
                                     pNew3DCurve,    // out: 3d intersection curve
                                     pUVCurve1,      // out: associated UVTrimCurve on 1st surface
                                     pUVCurve2,      // out: associated UVTrimCurve on 2nd surface
                                     FALSE,          // in : bOptSkipTwoPointsIntersection
                                     NULL,           // in : reference point
                                     iDebugLevel )); // in : TRUE = skip trying cheap sm_TwoPntsIntersection() 
                                                     //             before moving onto expensive general surf/surf xSect solver
                                                     //      FALSE= try cheap sm_TwoPntsIntersection() before
                                                     //             resorting to expensive general surf/surf xSect solver


          if (pUVCurve2) { delete pUVCurve2; pUVCurve2 = NULL ; }
          pFilletEdge->SetCurve(pNew3DCurve, FALSE) ; // FALSE = don't delete preExisting Curve - expect pFilletEdge->Curve == NULL
                                                      // side effect: delete current pEdge->UVTrimCurves
          pNew3DCurve->SetOwner(pFilletEdge);
          pPrimEU->SetUVCurve(pUVCurve1);
        }
    } // end Planar FilletCorner branch
  else if (bCreateSurfOfRevo) 
    {
      // create SurfOfRevolution to fill Loop of BoundaryEdges - if possible
      if(SM_SUCCESS != sm_CreateSrfOfRevolution(m_crContext,           // in : context for new object construction
                                                sCurves,               // in : CCW loop of curves to fill with SurfOfRevolution
                                                lMajorArcIndex,        // in : Index of ArcCurve that defines AxisOfRotation and ArcAngle
                                                m_dThisApproxTol3d,    // in : MaxAllowed gap between NewSurface and boundary Curves
                                                pSurface,              // out: SurfOfRevolution when BoundaryCurves can be Approximated, else NULL
                                                iDebugLevel ))         // in : GreaterThan 0 = Add MajorArc Graphics to current display in debug mode
        {
          // when SurfOfRevolution could not be found to fill Boundary Curves 
          // try to convert it to a variable-radius fillet to achive better G1 continuity. 
          if(SM_SUCCESS != MakeVariableRadiusFillet(pSurface)) 
            {
              // when both tries fail - quit
              return FALSE;
            }
        }
    } // end Try SurfOFRevolution branch
  else 
    {
      // Try to create a sphere that fills boundary sCurves to within tolerance
      SmTArray<SmCurve*>     sCCWArcs;
      SmTArray<SmOrientType> sOrients;
      SmBSplineSurface *pBSPSphere = NULL ;
      if(SM_SUCCESS == SmBSplineSurface::CreateSphereFromArcs(m_crContext,
                                                              sCurves,
                                                              m_dThisApproxTol3d,
                                                              sCCWArcs,
                                                              sOrients,
                                                              pBSPSphere)) 
        { pSurface = pBSPSphere ; }
      else // SmBSplineSurface::CreateSphereFromArcs failed
        {
          // When a Sphere could not be found to fill Boundary Curves 
          // try to convert it to a variable-radius fillet to achive better G1 continuity.
          if(SM_SUCCESS != MakeConstantRadiusFillet(pSurface)) 
            {
              // when both tries fail - quit
              // if(pSurface) { delete pSurface; pSurface = NULL;}
              return FALSE;
            }
        }
    }

  // save newSurface
  m_vSurfaces.Add(pSurface);

#ifdef SM_DEBUG_CODE
  ULONG di ;
  if ( DebugLevel() > 0 ) 
    {
      if (FALSE) 
        { smgfx_Erase(); }
      smgfx_SetLook(1,2, 1,0,0); for (di=0; di<m_vEdges.GetSize(); di++) { SmCurve * pCurve = m_vEdges[di]->GetCurve();
                                                                           SmExtent1d sIvl = pCurve->GetNaturalInterval();
                                                                           pCurve->DrawWDeriv(sIvl); sm_GraphicsLoop();
                                                                         }
      smgfx_SetLook(1,2, 1,0,0); pSurface->DrawUV(5,5); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE
  
  // all done
  return TRUE;

} // end SmFilletCorner::MakeAnalyticCornerSurface

/*******************************************************************//**
PURPOSE: Create Face (and Edges and Verts) in m_pFilletBrep from 
         m_vSurfaces with call m_pFilletBrep->MakeFaceWithCurves()

NOTES:   no Faces made when m_vSurfaces is empty
***********************************************************************/
SmStatus SmFilletCorner::MakeFaceBrep()
{
  // locals: originalBrep and surface count
  SmBrep * pFilletBrep  = m_pExecutive->m_pFilletBrep;
  ULONG    lNumSurfaces = m_vSurfaces.GetSize();
  ULONG    ii;

  // no work - no surface for which to build a face
  if (lNumSurfaces == 0)
    { return SM_SUCCESS; }

#ifdef SM_DEBUG_CODE
  // draw all m_Edges->Curves(red) 
  if ( DebugLevel() > 0 ) 
    {
      // if (0)
      //   { smgfx_Erase(); }

      smgfx_SetLook(3,5, 1,0,0);
      for (ULONG ij=0; ij<m_vEdges.GetSize(); ij++)
        {
          SmCurve * p3DCurve = m_vEdges[ij]->GetCurve();
          p3DCurve->Draw(); sm_GraphicsLoop();
        }
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // when face is defined by just 1 surface
  if (lNumSurfaces == 1)
    {
      // locals
      SmSurface              * pSurface    = m_vSurfaces[0];
      ULONG                    lTotalEdges = m_vEdges.GetSize();
      SmRegion               * pNewRegion = NULL;
      SmShell                * pNewShell = NULL;
      SmFace                 * pNewFace = NULL;
      SmTArray<SmPoint3d>      sLoopPoints;
      SmTArray<ULONG>          sCurveLoops;
      SmTArray<SmOrientType>   sCurveOrients;
      SmTArray<SmCurve*>       s3DCurves;

      // set number edges in newFace's loop
      sCurveLoops.Add(lTotalEdges);

      // set s3Dcurves and sCurveOrients with every FilletCorner->FilletEdge
      for (ii=0; ii<lTotalEdges; ii++)
        {
          SmFilletEdge * pCurrEdge = m_vEdges[ii];
          SmCurve      * p3DCurve = NULL;
          SER(pCurrEdge->GetCurve()->Copy(m_crContext,p3DCurve));

#ifdef SM_DEBUG_CODE
          // draw filletEdge->curve(blue)
          if ( DebugLevel() > 0 )
            {
              smgfx_SetLook(5,7, 0,0,1); p3DCurve->Draw(); sm_GraphicsLoop();
              sm_GraphicsLoop();
            }
#endif // SM_DEBUG_CODE
          s3DCurves.Add(p3DCurve);
          sCurveOrients.Add(SM_OT_SAME);
        }  // end iter every filletCorner->FilletEdge

      SmOrientType eLoopOrient = SM_OT_SAME;
      {
        // Do a simple test to determine the loop orientation
        SmVector3d sTangent1, sTangent2, sPatchNormal;

        // let patchNormal = crossProduct of two edge endTangents at one vertex
        SER(m_vEdges[1]->GetEndTangent(FALSE,sTangent1));
        SER(m_vEdges[2]->GetEndTangent(TRUE,sTangent2));
        sPatchNormal = sTangent1 * sTangent2;
        sPatchNormal.Unitize();

        // get Surface UVPoint for vertex location
        SmPoint3d  sPnt      = m_vEdges[2]->GetVertex()->GetPoint();
        SmExtent2d sUVDomain = pSurface->GetNaturalUVDomain();
        SmSolutionArray sSolutions;
        SER(pSurface->GlobalPointSolve(sUVDomain,
                                       SM_SO_MINIMIZE,
                                       sPnt,
                                       SM_EFF_ZERO,
                                       NULL,
                                       SM_SR_ALL,
                                       sSolutions));
        if (sSolutions.GetSize() < 1) SER(SM_ERR);  // != 1?
        SmPoint2d sUV = SmPoint2d(sSolutions[0].m_vStart[0],sSolutions[0].m_vStart[1]);

        // get Surface Normal at vertex location
        SmVector3d sSurfNormal;
        SER(pSurface->EvaluateNormal(sUV,TRUE,TRUE,sSurfNormal));
        sSurfNormal.Unitize();

        // when SurfaceNormal and PatchNormal are aligned eLoopOrient = SM_OT_SAME
        if (sSurfNormal.Dot(sPatchNormal) < -SM_EFF_ZERO)
          {  eLoopOrient = SM_OT_OPPOSITE;
          }

      } // end set eLoopOrient value block
      // Make a face bounded by m_vEdges with m_vSurfaces[0] shape
      //  in pFilletBrep's infinite region.
      SER(pFilletBrep->MakeFaceWithCurves
             (NULL,                             // in : NULL: infinite region
              sCurveLoops,                      // in : 1 entry per loop, value = loop edge count, 1st entry=outer loop
              &s3DCurves,                       // in : opt ordered 3d trimming curves assigned to loops per sLoopEUCounts
              NULL,                             // in : opt ordered 2d trimming curves assigned to loops per sLoopEUCounts
              sCurveOrients,                    // in : associated orients for each trimming curve, SM_OT_SAME or SM_OT_OPPOSITE
              sLoopPoints,                      // in : Point positions to build SmVertex VertexLoops
              pSurface,                         // in : new face->Surface
              pSurface->GetNaturalUVDomain(),   // in : domain of Surface used by face
              eLoopOrient,                      // in : Surface orient, oneof SM_OT_SAME or SM_OT_OPPOSITE
              pNewRegion,                       // out: New region if any. NULL when building trimmed surfaces, may be NotNULL for solids.
              pNewShell,                        // out: New shell if any.  Trimmed surfaces always create a new shell.
              pNewFace));                       // out: the new face

#ifdef SM_DEBUG_CODE
      // draw newFace(red)
      if ( DebugLevel() > 0 )
        {
          smgfx_SetLook(1,2, 1,0,0);
          pNewFace->Draw(SM_DM_WIREFRAME); sm_GraphicsLoop();
          pNewFace->Draw(SM_DM_CROSSHATCH,40,40); sm_GraphicsLoop();
          sm_GraphicsLoop();
          pFilletBrep->Dump();
        }
#endif // SM_DEBUG_CODE
      // all done
      return SM_SUCCESS;

    } // end building just 1 face check

  // arrive here when building an N-sided patch

  // for every m_Surfaces->Surface - build 1 face with natural edge boundaries
  for (ii=0; ii<lNumSurfaces; ii++)
    {
      SmSurface * pSurface = m_vSurfaces[ii];
      SmExtent2d sUVDomain = pSurface->GetNaturalUVDomain();

      SmTArray<SmCurve*>     s3DCurves;
      SmTArray<SmOrientType> sCurveOrients;

      // Add surface Natural IsoParameter boundaries to s3DCurves array
      SmBSplineCurve *pNewBSC = NULL ;
      SER(pSurface->CreateIsoParametricCurve(m_crContext, SM_SP_U, sUVDomain.GetMin().x, 0.0, pNewBSC));
      s3DCurves.Add(pNewBSC);
      sCurveOrients.Add(SM_OT_SAME);
      SER(pSurface->CreateIsoParametricCurve(m_crContext, SM_SP_V, sUVDomain.GetMax().y, 0.0, pNewBSC));
      s3DCurves.Add(pNewBSC);
      sCurveOrients.Add(SM_OT_SAME);
      SER(pSurface->CreateIsoParametricCurve(m_crContext, SM_SP_U, sUVDomain.GetMax().x, 0.0, pNewBSC));
      s3DCurves.Add(pNewBSC);
      sCurveOrients.Add(SM_OT_OPPOSITE);
      SER(pSurface->CreateIsoParametricCurve(m_crContext, SM_SP_V,sUVDomain.GetMin().y, 0.0, pNewBSC));
      s3DCurves.Add(pNewBSC);
      sCurveOrients.Add(SM_OT_OPPOSITE);

      SmRegion *pNewRegion = NULL;
      SmShell  *pNewShell = NULL;
      SmFace   *pNewFace = NULL;
      SmTArray<SmPoint3d> sLoopPoints;
      SmTArray<ULONG> sCurveLoops;
      sCurveLoops.Add(4); // Each rectangular surface has 4 sides

      SmOrientType eLoopOrient = SM_OT_SAME;
      {
        // Do a simple test to determine the loop orientation
        SmVector3d sTangent1, sTangent2, sPatchNormal;

        // let patchNormal = crossProduct of two edge endTangents at one vertex
        SmVector3d sPV1[2];
        SmCurve * pCrv1 = s3DCurves[0];
        SmCurve * pCrv2 = s3DCurves[1];
        SER(pCrv1->Evaluate(pCrv1->GetNaturalInterval().GetMax(),1,TRUE,sPV1));
        sTangent1 = sPV1[1];
        SER(pCrv2->Evaluate(pCrv2->GetNaturalInterval().GetMin(),1,TRUE,sPV1));
        sTangent2 = sPV1[1];
        sPatchNormal = sTangent1 * sTangent2;
        sPatchNormal.Unitize();

        // get Surface UVPoint for vertex location
        SmPoint3d sPnt = sPV1[0];
        SmExtent2d sDomain = pSurface->GetNaturalUVDomain();
        SmSolutionArray sSolutions;
        SER(pSurface->GlobalPointSolve(sDomain,SM_SO_MINIMIZE,
            sPnt,SM_EFF_ZERO,NULL,SM_SR_ALL,sSolutions));
        if (sSolutions.GetSize() < 1) SER(SM_ERR);// != 1?
        SmPoint2d sUV = SmPoint2d(sSolutions[0].m_vStart[0],sSolutions[0].m_vStart[1]);

        // get Surface Normal at vertex location
        SmVector3d sSurfNormal;
        SER(pSurface->EvaluateNormal(sUV,TRUE,TRUE,sSurfNormal));
        sSurfNormal.Unitize();

        // when SurfaceNormal and PatchNormal are aligned eLoopOrient = SM_OT_SAME
        if (sSurfNormal.Dot(sPatchNormal) < -SM_EFF_ZERO)
            eLoopOrient = SM_OT_OPPOSITE;

      } // end set eLoopOrient value block

      // make a face bounded by m_vEdges with m_vSurfaces[0] shape
      //  in pFilletBrep's infinite region.
      SER(pFilletBrep->MakeFaceWithCurves(NULL,                            // in : use infinite region
                                          sCurveLoops,                     // in : 1 entry per loop, value = loop edge count, 1st entry=outer loop
                                          &s3DCurves,                      // in : opt ordered 3d trimming curves assigned to loops per sLoopEUCounts
                                          NULL,                            // in : opt ordered 2d trimming curves assigned to loops per sLoopEUCounts
                                          sCurveOrients,                   // in : associated orients for each trimming curve, SM_OT_SAME or SM_OT_OPPOSITE
                                          sLoopPoints,                     // in : Point positions to build SmVertex VertexLoops
                                          pSurface,                        // in : new face->Surface
                                          pSurface->GetNaturalUVDomain(),  // in : domain of Surface used by face
                                          eLoopOrient,                     // in : Surface orient, oneof SM_OT_SAME or SM_OT_OPPOSITE
                                          pNewRegion,                      // out: New region if any. NULL when building trimmed surfaces, may be NotNULL for solids.
                                          pNewShell,                       // out: New shell if any.  Trimmed surfaces always create a new shell.
                                          pNewFace));                      // out: the new face

#ifdef SM_DEBUG_CODE
      // draw newFace(red), Surface(black)
      if ( DebugLevel() > 0 )
        {
          // draw face
          smgfx_SetLook(1,2, 1,0,0); pNewFace->Draw(SM_DM_WIREFRAME);     sm_GraphicsLoop();
          smgfx_SetLook(1,2, 1,0,0); pNewFace->Draw(SM_DM_CROSSHATCH);     sm_GraphicsLoop();

          // draw surface
          smgfx_SetLook(1,2, 0,0,0); pSurface->DrawUV(5,5); sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE
    } // end iter every surface buiding a face

  return SM_SUCCESS;

} // end SmFilletCorner::MakeFaceBrep

/*******************************************************************//**
PURPOSE: Clear all geometry that has been created for each corner edge

NOTES:
***********************************************************************/
void SmFilletCorner::ReInitializeCornerEdges()
{
  // for every corner SmFilletEdge
  ULONG ii;
  for (ii=0; ii<m_vEdges.GetSize(); ii++) 
    {
      SmFilletEdge * pFilletEdge = m_vEdges[ii];

      pFilletEdge->SetCurve(NULL, TRUE) ; // TRUE = delete preExisting Curve
                                          // side effect: delete current pEdge->UVTrimCurves

      //  // gwc: the following block to check and delete UVTrimCurves should now be obsolete - no harm in the extra check
      //  SmFilletEdgeuse * pPrimEU   = (SmFilletEdgeuse*)pFilletEdge->GetPrimaryEdgeuse();
      //  SmFilletEdgeuse * pMateEU   = (SmFilletEdgeuse*)pPrimEU->GetMate();
      //  SmBSplineCurve  * pUVCurve1 = pPrimEU->GetUVTrimCurvePointer();
      //  SmBSplineCurve  * pUVCurve2 = pMateEU->GetUVTrimCurvePointer();
      //  
      //  if (pUVCurve1) { delete pUVCurve1; pUVCurve1 = NULL ;
      //                   pPrimEU->SetUVCurve(NULL);
      //                 }
      //  if (pUVCurve2) { delete pUVCurve2; pUVCurve2 = NULL ;
      //                   pMateEU->SetUVCurve(NULL);
      //                 }
    } // end iter every corner SmFilletEdge

} // end SmFilletCorner::ReInitializeCornerEdges

/*******************************************************************//**
PURPOSE: If a Face is split, and we are on that Face, see whether we should switch to the new Face.

NOTES: Check our FilletVertices and FilletEdges.
***********************************************************************/
SmStatus SmFilletCorner::UpdateSplitFace( SmFace* pOrigFace, SmFace *pSplitFace )
{
  SmTArray< SmFilletVertex* > sFVs;
  this->GetFilletVertices( sFVs );
  for ( ULONG ii=0; ii<sFVs.GetSize(); ii++ )
  {
      sFVs[ii]->UpdateSplitFace( pOrigFace, pSplitFace );
  }

  SmTArray< SmFilletEdge* > sFEs;
  this->GetFilletEdges( sFEs );
  for ( ULONG ii=0; ii<sFEs.GetSize(); ii++ )
  {
      sFEs[ii]->UpdateSplitFace( pOrigFace, pSplitFace );
  }

  return SM_SUCCESS;

} // endf SmFilletCorner::UpdateSplitFace

/*******************************************************************//**
PURPOSE: Tell the FilletExecutive about a fillet error.

USAGE ---
    This could arise from failure to intersect two fillet surfaces.
***********************************************************************/
void SmFilletCorner::RecordFilletError
 (SmFilletErrorType  eErrorType,      // in : 
  SmSurface        * pFilletSurface1, // in : 
  SmSurface        * pFilletSurface2, // in : 
  SmFilletVertex   * pStartFV,        // in : 
  SmFilletVertex   * pEndFV,          // in : 
  const TCHAR      * cComment)        // in : 
{
  TCHAR      pName[SM_TBLOCK_SIZE];
  SmVertex * pOrigVtx = SM_CONST_CAST( SmVertex*, this->GetFilletedVertex() );
  this->GetName( pName, SM_TBLOCK_SIZE );

 // Pass the call along to the FilletExectutive
  GetFilletExecutive()->NoteFilletError( eErrorType,
                                        GetCornerType(), 
                                        GetStatus(), 
                                        pName, 
                                        pOrigVtx,
                                        pFilletSurface1, 
                                        pFilletSurface2, 
                                        pStartFV, 
                                        pEndFV, 
                                        (TCHAR *)cComment );
} // end SmFilletCorner::RecordFilletError

/*******************************************************************//**
PURPOSE: Return the index of a given FilletVertex in this FilletCorner.

NOTES: Used in debugging.
    Returns error 9999 if the given FilletVertex is not in our list.
***********************************************************************/
ULONG SmFilletCorner::FindVertexIndex
 (const SmFilletVertex *pFilVtx ) 
 const
{
  ULONG lIdx;
  if ( m_vVertices.FindElement( SM_CONST_CAST(SmFilletVertex*, pFilVtx), lIdx ) )
    { return lIdx; }

  return 9999;  // Error return

} // SmFilletCorner::FindVertexIndex

/*******************************************************************//**
PURPOSE: Return the index of a given FilletEdge in this FilletCorner.

NOTES: Used in debugging.
    Returns error 9999 if the given FilletEdge is not in our list.
***********************************************************************/
ULONG SmFilletCorner::FindEdgeIndex
 (const SmFilletEdge *pFilEdge) 
 const
{

  ULONG lIdx;
  if ( m_vEdges.FindElement( SM_CONST_CAST(SmFilletEdge*, pFilEdge), lIdx ) )
    { return lIdx; }

  return 9999;  // Error return

} // end SmFilletCorner::FindEdgeIndex

/*******************************************************************//**
PURPOSE: Return the index of this FilletCorner in our FilletExecutive.

NOTES: Used in debugging.
    Returns error 9999 if 'this' is not in our FilletExecutive.
***********************************************************************/
ULONG SmFilletCorner::FindIndexInExec() const
{
  if ( m_pExecutive == NULL ) 
    { return 9999; }

  return m_pExecutive->FindCornerIndex( this );

} // end SmFilletCorner::FindIndexInExec

/*******************************************************************//**
PURPOSE: Find and return all SmFilletSolvers connected to this FilletCorner.

NOTES:
***********************************************************************/
void SmFilletCorner::GetFilletSolvers( SmTArray<SmFilletSolver*> & rSolvers )
{
  rSolvers.ReSet();

  for ( ULONG ii=0; ii<m_pSolverEUs->GetSize(); ii++ )
  {
      rSolvers.Add( m_pExecutive->GetFilletSolverOfEdgeuse( (*m_pSolverEUs)[0] ) );
  }
} // end SmFilletCorner::GetFilletSolvers

/*******************************************************************//**
PURPOSE:
NOTES:
***********************************************************************/
SmBoolean SmFilletCorner::IsKindOf( SM_TYPE t ) const
{
  return ((SmFilletCorner_TYPE == t) ? TRUE : SmObject::IsKindOf( (t) ));
}

/*******************************************************************//**
PURPOSE: Debug dump for SmFilletCorner
***********************************************************************/
void SmFilletCorner::Dump() const { DumpLevel( 0, NULL ); }

/*******************************************************************//**
PURPOSE: Debug dump for SmFilletCorner

NOTES:  iDebugLevel <   5 - no Pretty Print
        iDebugLevel >=  5 - pretty print FilletCorner type
        iDebugLevel >= 10 - pretty print Every FilletCorner, Every FilletEdge, and Num of FilletSurfaces
        iDebugLevel >= 20 - pretty print Every FilletSurface(bAbbrev = TRUE)
        iDebugLevel >= 60 - pretty print Every FilletSurface(bAbbrev = FALSE)
***********************************************************************/
void SmFilletCorner::DumpLevel
 (int           iDebugLevel, 
  const TCHAR * cpMsg)  
 const
{
  TCHAR sBuff[SM_TBLOCK_SIZE];

  if ( cpMsg != NULL ) smos_WriteBuffer( cpMsg );

  // if ( iDebugLevel < 5 ) { return; }

  // header
  ULONG lIdx = m_pExecutive->FindCornerIndex( this );
  smos_sprintf( sBuff, _T("\nDump of SmFilletCorner [ %4lu ] "), lIdx );
  smos_WriteBuffer(sBuff);

  // Base Vertex
  m_cpVertex->Dump( _T("\n  Base Vertex: " ));

  GetName( sBuff, SM_TBLOCK_SIZE ) ; smos_WriteBuffer(sBuff);
  smos_sprintf( sBuff, _T("  Type:[%s]"), 
                GetCornerType() == SM_FCR_OPEN          ? _T("SM_FCR_OPEN")
              : GetCornerType() == SM_FCR_DEGENERATE    ? _T("SM_FCR_DEGENERATE")
              : GetCornerType() == SM_FCR_1_x_1         ? _T("SM_FCR_1_x_1")
              : GetCornerType() == SM_FCR_2_x_2         ? _T("SM_FCR_2_x_2")
              : GetCornerType() == SM_FCR_3_x_2_MIXED   ? _T("SM_FCR_3_x_2_MIXED")
              : GetCornerType() == SM_FCR_4_x_3         ? _T("SM_FCR_4_x_3")
              : GetCornerType() == SM_FCR_N_x_1_CLOSED  ? _T("SM_FCR_N_x_1_CLOSED")
              : GetCornerType() == SM_FCR_N_x_1         ? _T("SM_FCR_N_x_1")
              : GetCornerType() == SM_FCR_N_x_2         ? _T("SM_FCR_N_x_2")
              : GetCornerType() == SM_FCR_N_x_N         ? _T("SM_FCR_N_x_N")
              : GetCornerType() == SM_FCR_N_x_N_CONVEX  ? _T("SM_FCR_N_x_N_CONVEX")
              : GetCornerType() == SM_FCR_N_x_N_CONCAVE ? _T("SM_FCR_N_x_N_CONCAVE")
              :                                           _T("SM_FCR_UNKNOWN"));
  smos_WriteBuffer(sBuff);

  // Num of FilletVertices, FilletEdges, FilletSolvers
  ULONG ii, lNumFV = m_vVertices.GetSize() ;
  ULONG     lNumFE = m_vEdges.GetSize() ;
  ULONG     lNumFS = m_vSurfaces.GetSize() ;

  ULONG SmSurface_COUNT           = 0 ;
  ULONG SmBSplineSurface_COUNT    = 0 ;
  ULONG SmPlane_COUNT             = 0 ;
  ULONG SmCone_COUNT              = 0 ;
  ULONG SmCylinder_COUNT          = 0 ;
  ULONG SmSphere_COUNT            = 0 ;
  ULONG SmTorus_COUNT             = 0 ;
  ULONG SmSurfOfRevolution_COUNT  = 0 ;
  ULONG SmSurfOfExtrusion_COUNT   = 0 ;
  ULONG SmBlendSurface_COUNT      = 0 ;
  ULONG SmOffsetSurface_COUNT     = 0 ;
  ULONG SmSTEPSurface_COUNT       = 0 ;
  ULONG SmSrfInVolume_COUNT       = 0 ;
  ULONG lOtherSurface_COUNT       = 0 ;

  // count the surface types
  for(ii=0;ii<m_vSurfaces.GetSize();ii++)
    {
      SmSurface *pSurface = m_vSurfaces[ii] ;
      switch(pSurface->GetType())
        { case SmSurface_TYPE          : SmSurface_COUNT++ ;           break ;
          case SmBSplineSurface_TYPE   : SmBSplineSurface_COUNT++ ;    break ;
          case SmPlane_TYPE            : SmPlane_COUNT++ ;             break ;
          case SmCone_TYPE             : SmCone_COUNT++ ;              break ;
          case SmCylinder_TYPE         : SmCylinder_COUNT++ ;          break ;
          case SmSphere_TYPE           : SmSphere_COUNT++ ;            break ;
          case SmTorus_TYPE            : SmTorus_COUNT++ ;             break ;
          case SmSurfOfRevolution_TYPE : SmSurfOfRevolution_COUNT++ ;  break ;
          case SmSurfOfExtrusion_TYPE  : SmSurfOfExtrusion_COUNT++ ;   break ;
          case SmBlendSurface_TYPE     : SmBlendSurface_COUNT++ ;      break ;
          case SmOffsetSurface_TYPE    : SmOffsetSurface_COUNT++ ;     break ;
          case SmSTEPSurface_TYPE      : SmSTEPSurface_COUNT++ ;       break ;
          case SmSrfInVolume_TYPE      : SmSrfInVolume_COUNT++ ;       break ;
          default: lOtherSurface_COUNT++ ;
        } // enlOtherSurface_COUNT      sd switch on surface->GetType()
    } // end iter every surface

  ULONG SmCurve_COUNT                = 0 ;
  ULONG SmLine_COUNT                 = 0 ;
  ULONG SmConic_COUNT                = 0 ;
  ULONG SmCircle_COUNT               = 0 ;
  ULONG SmEllipse_COUNT              = 0 ;
  ULONG SmParabola_COUNT             = 0 ;
  ULONG SmHyperbola_COUNT            = 0 ;
  ULONG SmCompositeCurve_COUNT       = 0 ;
  ULONG SmBSplineCurve_COUNT         = 0 ;
  ULONG SmHermiteCurve_COUNT         = 0 ;
  ULONG SmOffsetCurve_COUNT          = 0 ;
  ULONG SmProjectedCurve_COUNT       = 0 ;
  ULONG SmCompositeCurveRegion_COUNT = 0 ;
  ULONG SmCrvInVolume_COUNT          = 0 ;
  ULONG lOtherCurve_COUNT            = 0 ;

  // count the curve types
  for(ii=0;ii<m_vEdges.GetSize();ii++)
    {
      SmCurve *pCurve = m_vEdges[ii]->GetCurve() ;
      if(pCurve)
        {  
          switch(pCurve->GetType())
            { case SmCurve_TYPE               : SmCurve_COUNT++ ;                break ;
              case SmLine_TYPE                : SmLine_COUNT++ ;                 break ;
              case SmConic_TYPE               : SmConic_COUNT++ ;                break ;
              case SmCircle_TYPE              : SmCircle_COUNT++ ;               break ;
              case SmEllipse_TYPE             : SmEllipse_COUNT++ ;              break ;
              case SmParabola_TYPE            : SmParabola_COUNT++ ;             break ;
              case SmHyperbola_TYPE           : SmHyperbola_COUNT++ ;            break ;
              case SmCompositeCurve_TYPE      : SmCompositeCurve_COUNT++ ;       break ;
              case SmBSplineCurve_TYPE        : SmBSplineCurve_COUNT++ ;         break ;
              case SmHermiteCurve_TYPE        : SmHermiteCurve_COUNT++ ;         break ;
              case SmOffsetCurve_TYPE         : SmOffsetCurve_COUNT++ ;          break ;
              case SmProjectedCurve_TYPE      : SmProjectedCurve_COUNT++ ;       break ;
              case SmCompositeCurveRegion_TYPE: SmCompositeCurveRegion_COUNT++ ; break ;
              case SmCrvInVolume_TYPE         : SmCrvInVolume_COUNT++ ;          break ;
              default: lOtherCurve_COUNT++ ;
            } // ensd switch on curve->GetType()
        } // end FilletCorner topology buile without geometry check
    } // end iter every curve

  // Vertex, Edge, and Surface counts
  smos_sprintf( sBuff, _T("\n   Fillet Vertices:[%lu], Fillet Edges:[%lu], FilletSolvers:[%lu] \n      "), lNumFV, lNumFE, lNumFS );
  smos_WriteBuffer(sBuff);

  // output surface/curve geometry type counts
  if(SmSurface_COUNT              > 0) { smos_sprintf(sBuff,_T("SmSurfaces:[%ld], "),            SmSurface_COUNT             ) ; smos_WriteBuffer(sBuff) ; }
  if(SmBSplineSurface_COUNT       > 0) { smos_sprintf(sBuff,_T("BSplineSurfaces:[%ld], "),       SmBSplineSurface_COUNT      ) ; smos_WriteBuffer(sBuff) ; }
  if(SmPlane_COUNT                > 0) { smos_sprintf(sBuff,_T("Planes:[%ld], "),                SmPlane_COUNT               ) ; smos_WriteBuffer(sBuff) ; }
  if(SmCone_COUNT                 > 0) { smos_sprintf(sBuff,_T("Cones:[%ld], "),                 SmCone_COUNT                ) ; smos_WriteBuffer(sBuff) ; }
  if(SmCylinder_COUNT             > 0) { smos_sprintf(sBuff,_T("Cylinders:[%ld], "),             SmCylinder_COUNT            ) ; smos_WriteBuffer(sBuff) ; }
  if(SmSphere_COUNT               > 0) { smos_sprintf(sBuff,_T("Spheres:[%ld], "),               SmSphere_COUNT              ) ; smos_WriteBuffer(sBuff) ; }
  if(SmTorus_COUNT                > 0) { smos_sprintf(sBuff,_T("Tori:[%ld], "),                  SmTorus_COUNT               ) ; smos_WriteBuffer(sBuff) ; }
  if(SmSurfOfRevolution_COUNT     > 0) { smos_sprintf(sBuff,_T("SurfOfRevolutions:[%ld], "),     SmSurfOfRevolution_COUNT    ) ; smos_WriteBuffer(sBuff) ; }
  if(SmSurfOfExtrusion_COUNT      > 0) { smos_sprintf(sBuff,_T("SurfOfExtrusions:[%ld], "),      SmSurfOfExtrusion_COUNT     ) ; smos_WriteBuffer(sBuff) ; }
  if(SmBlendSurface_COUNT         > 0) { smos_sprintf(sBuff,_T("BlendSurfaces:[%ld], "),         SmBlendSurface_COUNT        ) ; smos_WriteBuffer(sBuff) ; }
  if(SmOffsetSurface_COUNT        > 0) { smos_sprintf(sBuff,_T("OffsetSurfaces:[%ld], "),        SmOffsetSurface_COUNT       ) ; smos_WriteBuffer(sBuff) ; }
  if(SmSTEPSurface_COUNT          > 0) { smos_sprintf(sBuff,_T("STEPSurfaces:[%ld], "),          SmSTEPSurface_COUNT         ) ; smos_WriteBuffer(sBuff) ; }
  if(SmSrfInVolume_COUNT          > 0) { smos_sprintf(sBuff,_T("SrfInVolumes:[%ld], "),          SmSrfInVolume_COUNT         ) ; smos_WriteBuffer(sBuff) ; }

  if(m_vSurfaces.GetSize() > 0)
    { smos_sprintf(sBuff,_T("%s"),"   " ) ; smos_WriteBuffer(sBuff) ; }

  if(SmCurve_COUNT                > 0) { smos_sprintf(sBuff,_T("SmCurves:[%ld], "),              SmCurve_COUNT               ) ; smos_WriteBuffer(sBuff) ; }
  if(SmLine_COUNT                 > 0) { smos_sprintf(sBuff,_T("Lines:[%ld], "),                 SmLine_COUNT                ) ; smos_WriteBuffer(sBuff) ; }
  if(SmConic_COUNT                > 0) { smos_sprintf(sBuff,_T("Conics:[%ld], "),                SmConic_COUNT               ) ; smos_WriteBuffer(sBuff) ; }
  if(SmCircle_COUNT               > 0) { smos_sprintf(sBuff,_T("Circles:[%ld], "),               SmCircle_COUNT              ) ; smos_WriteBuffer(sBuff) ; }
  if(SmEllipse_COUNT              > 0) { smos_sprintf(sBuff,_T("Ellipses:[%ld], "),              SmEllipse_COUNT             ) ; smos_WriteBuffer(sBuff) ; }
  if(SmParabola_COUNT             > 0) { smos_sprintf(sBuff,_T("Parabolas:[%ld], "),             SmParabola_COUNT            ) ; smos_WriteBuffer(sBuff) ; }
  if(SmHyperbola_COUNT            > 0) { smos_sprintf(sBuff,_T("Hyperbolas:[%ld], "),            SmHyperbola_COUNT           ) ; smos_WriteBuffer(sBuff) ; }
  if(SmCompositeCurve_COUNT       > 0) { smos_sprintf(sBuff,_T("CompositeCurves:[%ld], "),       SmCompositeCurve_COUNT      ) ; smos_WriteBuffer(sBuff) ; }
  if(SmBSplineCurve_COUNT         > 0) { smos_sprintf(sBuff,_T("BSplineCurves:[%ld], "),         SmBSplineCurve_COUNT        ) ; smos_WriteBuffer(sBuff) ; }
  if(SmHermiteCurve_COUNT         > 0) { smos_sprintf(sBuff,_T("HermiteCurves:[%ld], "),         SmHermiteCurve_COUNT        ) ; smos_WriteBuffer(sBuff) ; }
  if(SmOffsetCurve_COUNT          > 0) { smos_sprintf(sBuff,_T("OffsetCurves:[%ld], "),          SmOffsetCurve_COUNT         ) ; smos_WriteBuffer(sBuff) ; }
  if(SmProjectedCurve_COUNT       > 0) { smos_sprintf(sBuff,_T("ProjectedCurves:[%ld], "),       SmProjectedCurve_COUNT      ) ; smos_WriteBuffer(sBuff) ; }
  if(SmCompositeCurveRegion_COUNT > 0) { smos_sprintf(sBuff,_T("CompositeCurveRegions:[%ld], "), SmCompositeCurveRegion_COUNT) ; smos_WriteBuffer(sBuff) ; }
  if(SmCrvInVolume_COUNT          > 0) { smos_sprintf(sBuff,_T("CrvInVolumes:[%ld], "),          SmCrvInVolume_COUNT         ) ; smos_WriteBuffer(sBuff) ; }

  smos_sprintf(sBuff,_T("%s"), _T("\n")) ; smos_WriteBuffer(sBuff);

  if ( iDebugLevel < 10 ) 
    { return; }

  // for every SmFilletVertex - Dump
  for(ii=0;ii<lNumFV;ii++)
    {
      smos_sprintf( sBuff, _T("\n   Fillet Vertex:[%lu of %lu] : "), ii, lNumFV);
      smos_WriteBuffer(sBuff);
      m_vVertices[ii]->DumpLevel( iDebugLevel - 10 );
    }

  // for every SmFilletEdge - Dump
  for(ii=0;ii<lNumFE;ii++)
    {
      smos_sprintf( sBuff, _T("\n   Fillet Edge:[%lu of %lu] : "), ii, lNumFE);
      smos_WriteBuffer(sBuff);
      m_vEdges[ii]->DumpLevel( iDebugLevel - 10 );
    }

  // when asked
  if ( iDebugLevel > 20 ) 
    {
      SmBoolean bAbbrev = ( iDebugLevel < 60 );

      // for every FilletSurface - Dump
      for(ii=0;ii<lNumFS;ii++)
        {
          smos_sprintf( sBuff, _T("\n   Fillet Solver:[%lu of %lu] : "), ii, lNumFS);
          smos_WriteBuffer(sBuff);
          m_vSurfaces[ii]->Dump( bAbbrev );
        }
    }

}  // end SmFilletCorner::DumpLevel()

/*******************************************************************//**
PURPOSE: Debug draw for SmFilletCorner

NOTES:  Draw Red   = OrigBrep Verts
             Green = FilletVertices
             Blue  = FilletEdges
             Cyan  = New Surfaces
***********************************************************************/
SmDisplayList * SmFilletCorner::Draw
 (SmGfxArraySet * pOptGfxSet)      // i/o: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.
                                   //      NULL to ignore. default:[NULL]
 const
{
  SmDisplayList *pRtn = NULL ;

#ifdef SM_GFX_CODE
  ULONG ii;

  // Get global SmGraphicsExtern.cpp:s_Disp display parameters
  // const SmDisplayParameters &rDisp = smgfx_RefGlobalDisplayParameters() ;

  // start new displayList (unless one is already open)
  SmVector3d sColor = smgfx_GetOutputColor(pOptGfxSet) ;
  smgfx_Open(smgfx_GetRuleColor(this), NULL, NULL, FALSE, pOptGfxSet);

  // output Corner Graphics
  smgfx_SetLook(1,6, 1,0,0); m_cpVertex->Draw(pOptGfxSet);  // OrigBrep Vertex
  smgfx_SetLook(1,6, 0,1,0); for ( ii = 0; ii < m_vVertices.GetSize(); ii++ ) // Corner FilletVertices [ordered] 
                               { m_vVertices[ii]->Draw(pOptGfxSet); }
  smgfx_SetLook(1,2, 0,0,1); for ( ii = 0; ii < m_vEdges.GetSize(); ii++ )    // Corner FilletEdges [ordered]
                               { m_vEdges[ii]->Draw(pOptGfxSet); }
  smgfx_SetLook(1,1, 0,1,1); for ( ii = 0; ii < m_vSurfaces.GetSize(); ii++ ) // New Surfaces to fill corner EndFaces and Holes
                               { m_vSurfaces[ii]->DrawUV(8,8,FALSE,NULL,FALSE,pOptGfxSet); }
  // end display list
  smgfx_OutputColor(sColor, pOptGfxSet) ;
  pRtn = smgfx_Close(pOptGfxSet) ;

#else
  SM_REF1(pOptGfxSet);
#endif // SM_GFX_CODE
  return(pRtn) ;

}  // end SmFilletCorner::Draw()

/*******************************************************************//**
PURPOSE: Create a 1x1 corner. Typically, when only one (closed & non-
    lamina) edge is connected to this corner vertex such that it separates
    two adjacent surfaces, a closed-fillet surface can be defined by rolling
    along this edge. (e.g. when two tori form an '8' shape, only one
    vertex and one edge are defined at the intersection)

NOTES: Will only handle the tangency cases for now.
***********************************************************************/
SmFillet1x1Corner::SmFillet1x1Corner
  (const SmContext      & crContext,
   const SmVertex       * cpVertex,
   SmTArray<SmEdgeuse*> * pSolverEUs,
   SmTArray<SmEdgeuse*> * pAllEUs,
   SmFilletExecutive    * pExec,
   double                 dApproxTol3d,
   double                 dTangencyTolRadians)
 : SmFilletCorner(crContext,
                  cpVertex,
                  pSolverEUs,
                  pAllEUs,
                  pExec,
                  dApproxTol3d,
                  dTangencyTolRadians)
{

} // end SmFillet1x1Corner::SmFillet1x1Corner

/*******************************************************************//**
PURPOSE: Create 1x1 corner by creating its boundary FilletCurves
         and FilletPoints in m_pPseudoBrep. 

NOTES: The topology of a 1x1 corner consists of only one fillet edge.
***********************************************************************/
SmStatus SmFillet1x1Corner::MakeCornerTopology
  ()
{
  // locals
  SmEdgeuse          * pFilEdgeuse = (*m_pSolverEUs)[0] ;
  SmFilletSolver     * pFilSolver  = GetFilletExecutive()->GetFilletSolverOfEdgeuse(pFilEdgeuse); NER(pFilSolver) ;
  SmFilletEdge       * pFilletEdge = NULL ;
  SmFilletEdgeuse    * pNewEdgeuse = NULL ;
  SmTArray<SmEdgeuse*> sFilEdgeuses ;

  // Create a fillet vertex on one side of the edge
  SmFilletVertex * pFV0   = new (m_pPseudoBrep) SmFilletVertex(this);
  SmFace         * pFace0 = pFilEdgeuse->GetFace();
  pFV0->SetPointClass(SM_PC_FACE, pFace0);
  pFV0->SetFilletVertexType(SM_FV_ON_CROSS_SECTION);
  m_vVertices.Add(pFV0);

  // Create another fillet vertex on the other side of the edge
  SmFilletVertex * pFV1   = new (m_pPseudoBrep) SmFilletVertex(this);
  SmFace         * pFace1 = pFilEdgeuse->GetRadial()->GetFace();
  pFV1->SetPointClass(SM_PC_FACE, pFace1);
  pFV1->SetFilletVertexType(SM_FV_MATE);
  pFV1->SetMate(0, pFV0);
  pFV0->SetMate(0, pFV1);
  m_vVertices.Add(pFV1);

  // Corner consists of one fillet edge only (joining pFV0 & pFV1)
  SER(SmFilletEdge::MakeFilletEdge(m_pPseudoBrep,    // in : target Brep to receive new topology objects
                                   pFV0,             // in : start of new FilletEdge
                                   pFV1,             // in : end   of new FilletEdge
                                   pFilletEdge,      // out: newly allocated FilletEdge - no Curve or UVTrimCurve data
                                   this));           // in : NewFilletEdge's filletCorner, gets stored in rpNewFilletEdge->m_cpCorner
                                                     //      NULL to ignore, default:[NULL]
  pFilletEdge->SetFilletEdgeType(SM_FE_CROSS_SECTION);

  //
  pFilletEdge->GetEdgeuses(sFilEdgeuses);
  if (sFilEdgeuses.GetSize() != 2) 
    { SER(SM_ERR); }

  //
  SmFilletEdgeuse * pPrimEU     = (SmFilletEdgeuse*) pFilletEdge->GetPrimaryEdgeuse();
  SmFilletGeom    * pFilletGeom = pFilSolver->GetFirstFilletGeom();
  SmFilletEdgeuse * pMateEU     = (SmFilletEdgeuse*)pPrimEU->GetMate();
  pPrimEU->SetFilletGeom(pFilletGeom);
  pMateEU->SetFilletGeom(pFilletGeom);
  m_vEdges.Add(pFilletEdge);

  // Setup rails of fillet
  ULONG lRailIndex = pFilSolver->FindIndexOfRailOnFace(pFace0);

  // Need two edgeuses for each (closed-)rail
  SER(pFilletGeom->MakeRailEdgeuse(m_pPseudoBrep, lRailIndex,   pFV0, pNewEdgeuse, SM_OT_SAME));
  SER(pFilletGeom->MakeRailEdgeuse(m_pPseudoBrep, lRailIndex,   pFV0, pNewEdgeuse, SM_OT_OPPOSITE));

  SER(pFilletGeom->MakeRailEdgeuse(m_pPseudoBrep, 1-lRailIndex, pFV1, pNewEdgeuse, SM_OT_SAME));
  SER(pFilletGeom->MakeRailEdgeuse(m_pPseudoBrep, 1-lRailIndex, pFV1, pNewEdgeuse, SM_OT_OPPOSITE));

  //
  m_bBlending = TRUE;

  // all done
  return SM_SUCCESS;

} // end SmFillet1x1Corner::MakeCornerTopology

/*******************************************************************//**
PURPOSE: If this FilletCorner has exactly two FilletSolvers, and their Edges
   meet with G1 continuity, return the other one than the passed-in argument.

NOTES:
   If the passed-in FilletSolver is not one of ours, return Null with an error.
   If there is only one FilletSolver, and it is the one passed in, return it.

   This class of FilletCorner has only one FilletSolver, and it is G1 closed.
***********************************************************************/
SmFilletSolver * SmFillet1x1Corner::GetTangentFilletSolver( const SmFilletSolver *pFS )
{
  if ( m_pSolverEUs->GetSize() != 1 ) { SM_ASSERT_ERR; return NULL; }

  SmEdgeuse      * pEU  = (*m_pSolverEUs)[0];
  SmFilletSolver *pMyFS = m_pExecutive->GetFilletSolverOfEdgeuse( pEU );

  if ( pMyFS != pFS ) { SM_ASSERT_ERR; return NULL; }

  return pMyFS;

} // end SmFillet1x1Corner::GetTangentFilletSolver

/*******************************************************************//**
PURPOSE: Create a 2x2 corner. When two edges are connected to this
    corner vertex with G1-continuity, a cross section can be defined to
    separate two G1-continuous fillets at this corner

NOTES: Will only handle the tangency cases for now.
***********************************************************************/
SmFillet2x2Corner::SmFillet2x2Corner
  (const SmContext      & crContext,
   const SmVertex       * cpVertex,
   SmTArray<SmEdgeuse*> * pSolverEUs,
   SmTArray<SmEdgeuse*> * pAllEUs,
   SmFilletExecutive    * pExec,
   double                 dApproxTol3d,
   double                 dTangencyTolRadians)
 : SmFillet1x1Corner(crContext,
                     cpVertex,
                     pSolverEUs,
                     pAllEUs,
                     pExec,
                     dApproxTol3d,
                     dTangencyTolRadians)
{

} // end SmFillet2x2Corner::SmFillet2x2Corner

/*******************************************************************//**
PURPOSE: Create 2x2 corner by specifying its boundary FilletCurves
    and FilletPoints.

NOTES:  The topology of a 2x2 corner consists of only one
    fillet edge.
***********************************************************************/
SmStatus SmFillet2x2Corner::MakeCornerTopology()
{
  SmEdgeuse * pBaseEdgeuse1 = (*m_pSolverEUs)[0];
  SmEdgeuse * pBaseEdgeuse2 = (*m_pSolverEUs)[1];
  SmFilletSolver * pFilSolver1 = GetFilletExecutive()->
      GetFilletSolverOfEdgeuse(pBaseEdgeuse1);
  SmFilletSolver * pFilSolver2 = GetFilletExecutive()->
      GetFilletSolverOfEdgeuse(pBaseEdgeuse2);
  NER(pFilSolver1); NER(pFilSolver2);
#ifdef SM_DEBUG_CODE
  if (FALSE) {
      smgfx_SetLook(1,2, 1,0,0); pBaseEdgeuse1->Draw(); sm_GraphicsLoop();
      smgfx_SetLook(1,2, 1,0,0); pBaseEdgeuse2->Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop();
  }
#endif // SM_DEBUG_CODE

  // Create a fillet vertex on one side of the edge
  SmFilletVertex * pFV0 = new (m_pPseudoBrep) SmFilletVertex(this);
  SmFace * pFace0 = pBaseEdgeuse1->GetFace();
  pFV0->SetPointClass(SM_PC_FACE, pFace0);
  pFV0->SetFilletVertexType(SM_FV_ON_CROSS_SECTION);
  m_vVertices.Add(pFV0);
  // Create another fillet vertex on the other side of the edge
  SmFilletVertex * pFV1 = new (m_pPseudoBrep) SmFilletVertex(this);
  SmFace * pFace1 = pBaseEdgeuse1->GetRadial()->GetFace();
  pFV1->SetPointClass(SM_PC_FACE, pFace1);
  pFV1->SetFilletVertexType(SM_FV_MATE);
  pFV1->SetMate(0, pFV0);
  pFV0->SetMate(0, pFV1);
  m_vVertices.Add(pFV1);

  // Corner consists of one fillet edge only (joining pFV0 & pFV1)
  SmFilletEdge * pFilletEdge = NULL;
  SER(SmFilletEdge::MakeFilletEdge(m_pPseudoBrep, // in : target Brep to receive new topology objects
                                   pFV0,          // in : start of new FilletEdge
                                   pFV1,          // in : end   of new FilletEdge
                                   pFilletEdge,   // out: newly allocated FilletEdge - no Curve or UVTrimCurve data
                                   this));        // in : NewFilletEdge's filletCorner, gets stored in rpNewFilletEdge->m_cpCorner
                                                  //      NULL to ignore, default:[NULL]
  pFilletEdge->SetFilletEdgeType(SM_FE_CROSS_SECTION);
  SmFilletEdgeuse * pPrimEU = (SmFilletEdgeuse*)
      pFilletEdge->GetPrimaryEdgeuse();
  SmFilletGeom * pFilletGeom1 = pFilSolver1->GetFirstFilletGeom();
  pPrimEU->SetFilletGeom(pFilletGeom1);
  SmFilletEdgeuse * pMateEU = (SmFilletEdgeuse*)pPrimEU->GetMate();
  SmFilletGeom * pFilletGeom2 = pFilSolver2->GetFirstFilletGeom();
  pMateEU->SetFilletGeom(pFilletGeom2);
  m_vEdges.Add(pFilletEdge);

  // Setup rails of fillet
  SmFilletEdgeuse * pNewEdgeuse = NULL;
  ULONG lRailIndex = pFilSolver1->FindIndexOfRailOnFace(pFace0);
  SER(pFilletGeom1->MakeRailEdgeuse(m_pPseudoBrep,
      lRailIndex,pFV0,pNewEdgeuse));
  SER(pFilletGeom1->MakeRailEdgeuse(m_pPseudoBrep,
      1-lRailIndex,pFV1,pNewEdgeuse));
  ULONG lOtherRailIndex = pFilSolver2->FindIndexOfRailOnFace(pFace0);
  SER(pFilletGeom2->MakeRailEdgeuse(m_pPseudoBrep,
      lOtherRailIndex,pFV0,pNewEdgeuse));
  SER(pFilletGeom2->MakeRailEdgeuse(m_pPseudoBrep,
      1-lOtherRailIndex,pFV1,pNewEdgeuse));

  m_bBlending = TRUE;

  return SM_SUCCESS;

} // end SmFillet2x2Corner::MakeCornerTopology

/*******************************************************************//**
PURPOSE: If this FilletCorner has exactly two FilletSolvers, and their Edges
   meet with G1 continuity, return the other one than the passed-in argument.

NOTES:
   If the passed-in FilletSolver is not one of ours, return Null with an error.
   If there is only one FilletSolver, and it is the one passed in, return it.

   This class of FilletCorner has two FilletSolvers, and they meet with G1 continuity.
***********************************************************************/
SmFilletSolver * SmFillet2x2Corner::GetTangentFilletSolver( const SmFilletSolver *pFS )
{
  if ( m_pSolverEUs->GetSize() != 2 )  { SM_ASSERT_ERR; return NULL; }

  SmFilletSolver * pFS1 = m_pExecutive->GetFilletSolverOfEdgeuse( (*m_pSolverEUs)[0] );
  SmFilletSolver * pFS2 = m_pExecutive->GetFilletSolverOfEdgeuse( (*m_pSolverEUs)[1] );

  if ( pFS1 == pFS )
    { return pFS2; }
  if ( pFS2 == pFS )
    { return pFS1; }

  SM_ASSERT_ERR; return NULL;

} // end SmFillet2x2Corner::GetTangentFilletSolver

/*******************************************************************//**
PURPOSE: A 3x2 corner whose two filleted edges has different edge convexity
    such as in L-shaped box case.

NOTES:
***********************************************************************/
SmFillet3x2MixedConvexityCorner::SmFillet3x2MixedConvexityCorner
  (const SmContext      & crContext,
   const SmVertex       * cpVertex,
   SmTArray<SmEdgeuse*> * pSolverEUs,
   SmTArray<SmEdgeuse*> * pAllEUs,
   SmFilletExecutive    * pExec,
   double                 dApproxTol3d,
   double                 dTangencyTolRadians)
 : SmFilletCorner(crContext,
                  cpVertex,
                  pSolverEUs,
                  pAllEUs,
                  pExec,
                  dApproxTol3d,
                  dTangencyTolRadians)
{

} // end SmFillet3x2MixedConvexityCorner::SmFillet3x2MixedConvexityCorner constructor


/*******************************************************************//**
PURPOSE: Calculate geometry of a 3x2MixedConvexityCorner patch and create 
   Face (and Edges and Verts) in m_pFilletBrep from m_vSurfaces with call 
   SmFilletCorner::CreateCornerPatch().

NOTES: Two of the fillet edges will be joined together to form
    a regular 3-sided COONS patch.
***********************************************************************/
SmStatus SmFillet3x2MixedConvexityCorner::CreateCornerPatch()
{
  // check state - 4 edges
  if (m_vEdges.GetSize() != 4)
    { SM_DBG_WARN(_T("Failed state check: 3x2 FilletCorner with other than 4 edges")) ;
      SER(SM_ERR);
    }

  // search for the onEdge and onExtended edges
  SmFilletEdge * pFEOnEdge         = NULL;
  SmFilletEdge * pFEOnExtendedEdge = NULL;
  ULONG ii, lFoundIndex = 0;
  for (ii=0; ii<4; ii++)
    {
      SmFilletEdge * pFE = m_vEdges[ii];
      switch (pFE->GetFilletEdgeType())
        {
          case SM_FE_ON_EXTEND_EDGE: pFEOnExtendedEdge = pFE;
                                     break;

          case SM_FE_ON_EDGE:        // To be deleted later
                                     lFoundIndex = ii;
                                     pFEOnEdge = pFE;
                                     break;
          case SM_FE_BLENDING_RAIL:
          case SM_FE_CROSS_SECTION:
          case SM_FE_CLIFF_RAIL:
          case SM_FE_FILLET_END:
          case SM_FE_FILLET_X_FILLET:
          case SM_FE_FILLET_X_SIDE_FACE:
          case SM_FE_PRECOMPUTED:
          case SM_FE_RAIL:
          case SM_FE_RAIL_EXTENSION:
          case SM_FE_RAIL_RAIL_INTERPOLATION:
          case SM_FE_SETBACK_RAIL:
          case SM_FE_SPLIT_FACE:
          case SM_FE_UNKNOWN:       break;

        } // end switch on filletEdge->Type
    } // end iter all 4 edges

  // check state - must have onEdge and onExtendedEdge
  NER(pFEOnEdge) ;
  NER(pFEOnExtendedEdge) ;

  // get onEdge->Vertices
  SmFilletVertex * pCornerV     = (SmFilletVertex*)pFEOnEdge->GetVertex();
  SmVertex       * pSurvivingFV = pFEOnEdge->GetOtherVertex(pCornerV);

  // check state
  if (   pCornerV->GetFilletVertexType() != SM_FV_ON_VERTEX
      || pFEOnExtendedEdge->GetVertex()  == pCornerV)
    {
      // Internal error, check corner topology
      SM_DBG_WARN(_T("3x2MixedConvexityCorner has confused topology")) ;
      SER(SM_ERR);
    }

  // get onEdge and onExtended curves
  SmBSplineCurve * pCurve1 = SM_CAST_PTR(SmBSplineCurve, pFEOnExtendedEdge->GetCurve());  NER(pCurve1);
  SmBSplineCurve * pCurve2 = SM_CAST_PTR(SmBSplineCurve, pFEOnEdge->GetCurve());  NER(pCurve2);

  // join the 2 curves
  SER(pCurve1->JoinWith(1,pCurve2,0));

#ifdef SM_DEBUG_CODE
  // draw the joined curve
  if ( DebugLevel() > 0 )
    {
      smgfx_SetLook(1,2, 1,0,0); pCurve1->Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // Combine two edges
  m_vEdges.RemoveAt(lFoundIndex);
  if (m_vVertices.FindElement(pCornerV,lFoundIndex) == FALSE)
    {
      SER(SM_ERR);
    }
  m_vVertices.RemoveAt(lFoundIndex);

  SM_ASSERT(pFEOnEdge != NULL) ; delete pFEOnEdge ; pFEOnEdge = NULL ;

  SmTArray<SmVertexuse*> sVUs;
  pCornerV->GetVertexuses(sVUs);
  for (ULONG j=0; j<sVUs.GetSize(); j++)
    {
      SmFilletVertexuse * pVU = (SmFilletVertexuse*)sVUs[j];
      //SmFilletEdge      * pFE = (SmFilletEdge*)pVU->GetEdgeuse()->GetEdge();
      pCornerV->Remove(pVU);
      SER(pSurvivingFV->PostInsert(pVU));
    }

  SM_ASSERT(pCornerV != NULL) ; delete pCornerV ; pCornerV = NULL ;

  // Project pFEOnExtendedEdge onto original face
  SmFace    * pOrigFace = pFEOnExtendedEdge->GetOriginalFace();
  NER(pOrigFace);
  SmSurface * pSurface  = pOrigFace->GetSurface();
  double dMaxDist = 0;
  double dDeviation = 0;
  SmTArray<SmBSplineCurve*> sUVCurves;
  SER(pSurface->DropCurve(m_crContext,
                          pSurface->GetNaturalUVDomain(),
                         *pCurve1,pCurve1->GetNaturalInterval(),
                          m_dThisApproxTol3d,
                          dMaxDist,       // out: MaxDist(DropPt, 3dCurvePt), found by sampling, may be slightly less than actual max.
                          dDeviation,     // out: MaxDist(DropPt->SrfNormalLine, 3dCurvePt), Quality Measure: expected to be close to 0.0
                          sUVCurves));
  if (sUVCurves.GetSize() != 1)
    {
      SmObjsDelete<SmBSplineCurve*> sDelCurves(&sUVCurves);
      SER(SM_ERR); // Unknown problem
    }
  SmBSplineCurve  * pNewPSCurve = sUVCurves[0];
  SmFilletEdgeuse * pMateEU     = (SmFilletEdgeuse*)pFEOnExtendedEdge->GetPrimaryEdgeuse()->GetMate();
  pMateEU->SetUVCurve(pNewPSCurve);

  // Create a 3-sided corner patch
  SER(SmFilletCorner::CreateCornerPatch());

  return SM_SUCCESS;

} // end SmFillet3x2MixedConvexityCorner::CreateCornerPatch

/*******************************************************************//**
PURPOSE: Create 3x2MixedConvexityCorner by specifying its boundary
    FilletCurves and FilletPoints.

NOTES: This corner should have 4 boundary edges
***********************************************************************/
SmStatus SmFillet3x2MixedConvexityCorner::MakeCornerTopology
  ()
{
int iDebugLevel = 0;
#ifdef SM_DEBUG_CODE
iDebugLevel = DebugLevel();
#endif // SM_DEBUG_CODE

    if (m_pAllEUs->GetSize() != 3) {
        SER(SM_ERR);
    }
    SmEdgeuse * pConvexEdgeuse = (*m_pSolverEUs)[0];
    SmEdgeuse * pConcaveEdgeuse = (*m_pSolverEUs)[1];
    if (pConcaveEdgeuse->IsConvexRadialSector(5) == FALSE) {
        SM_SWAP_PTR(SmEdgeuse,pConcaveEdgeuse,pConvexEdgeuse);
    }

    // Get Solvers
    SmFilletExecutive * pExec = GetFilletExecutive();
    SmFilletSolver * pConvexSolver = pExec->GetFilletSolverOfEdgeuse(pConvexEdgeuse);
    SmFilletSolver * pConcaveSolver = pExec->GetFilletSolverOfEdgeuse(pConcaveEdgeuse);
    SmFilletGeom * pConvexFG = pConvexSolver->GetFirstFilletGeom();
    SmFilletGeom * pConcaveFG = pConcaveSolver->GetFirstFilletGeom();

    SmEdgeuse * pCurrEU = NULL;
    for (ULONG i=0; i<3; i++) {
        SmEdgeuse * pEU = (*m_pAllEUs)[i];
        if (pEU != pConvexEdgeuse && pEU != pConcaveEdgeuse) {
            pCurrEU = pEU;
            break;
        }
    }
    NER(pCurrEU);

    ULONG lRailIndex, lOtherRailIndex;
    // This edgeuse is 'in-between' two filletted edgeuses
    SmFilletVertex * pFV0 = new (m_pPseudoBrep) SmFilletVertex(this);
    SmEdgeuse * pSideEU = sm_FindSideEdgeuse(pConcaveEdgeuse, pCurrEU, iDebugLevel );
    lRailIndex = pConcaveSolver->FindIndexOfRailXSideEdgeuse(pSideEU);
    SmFilletEdgeuse * pNewEdgeuse = NULL;
    SER(pConcaveFG->MakeRailEdgeuse(m_pPseudoBrep,lRailIndex,pFV0,pNewEdgeuse));
    pFV0->SetPointClass(SM_PC_EDGEUSE, pSideEU);
    pFV0->SetFilletVertexType(SM_FV_RAIL_X_EDGEUSE);
    m_vVertices.Add(pFV0);

    SmFace * pCommonFace = sm_GetFaceByTwoEdgeuses(
        pConvexEdgeuse,pConcaveEdgeuse,pCurrEU);
    SmFilletVertex * pFV1 = new (m_pPseudoBrep) SmFilletVertex(this);
    pFV1->SetPointClass(SM_PC_FACE, pCommonFace);
    pFV1->SetFilletVertexType(SM_FV_RAIL_X_RAIL);
    m_vVertices.Add(pFV1);

    // Setup rails of fillet
    lRailIndex = pConcaveSolver->FindIndexOfRailOnFace(pCommonFace);
    SER(pConcaveFG->MakeRailEdgeuse(m_pPseudoBrep,lRailIndex,pFV1,pNewEdgeuse));
    lOtherRailIndex = pConvexSolver->FindIndexOfRailOnFace(pCommonFace);
    SER(pConvexFG->MakeRailEdgeuse(m_pPseudoBrep,lOtherRailIndex,pFV1,pNewEdgeuse));

    // Make a cross section on the concave fillet
    SmFilletEdge * pFE0 = NULL;
    SER(SmFilletEdge::MakeFilletEdge(m_pPseudoBrep, // in : target Brep to receive new topology objects
                                     pFV0,          // in : start of new FilletEdge
                                     pFV1,          // in : end   of new FilletEdge
                                     pFE0,          // out: newly allocated FilletEdge - no Curve or UVTrimCurve data
                                     this));        // in : NewFilletEdge's filletCorner, gets stored in rpNewFilletEdge->m_cpCorner
                                                    //      NULL to ignore, default:[NULL]
    pFE0->SetFilletEdgeType(SM_FE_CROSS_SECTION);
    SmFilletEdgeuse * pPrimEU = (SmFilletEdgeuse*)pFE0->GetPrimaryEdgeuse();
    pPrimEU->SetFilletGeom(pConcaveFG);
    m_vEdges.Add(pFE0);

    SmFilletVertex * pFV2 = new (m_pPseudoBrep) SmFilletVertex(this);
    pFV2->SetFilletVertexType(SM_FV_RAIL_X_EXTENDED_EDGEUSE);
    pSideEU = sm_FindSideEdgeuse(pConvexEdgeuse,pCurrEU, iDebugLevel );
    lRailIndex = pConvexSolver->FindIndexOfRailXSideEdgeuse(pSideEU);
    SER(pConvexFG->MakeRailEdgeuse(m_pPseudoBrep,lRailIndex,pFV2,pNewEdgeuse));
    pFV2->SetPointClass(SM_PC_EDGEUSE, pSideEU);
    m_vVertices.Add(pFV2);

    // Make a cross section on the conveex fillet
    SmFilletEdge * pFE1 = NULL;
    SER(SmFilletEdge::MakeFilletEdge(m_pPseudoBrep, // in : target Brep to receive new topology objects
                                     pFV1,          // in : start of new FilletEdge
                                     pFV2,          // in : end   of new FilletEdge
                                     pFE1,          // out: newly allocated FilletEdge - no Curve or UVTrimCurve data
                                     this));        // in : NewFilletEdge's filletCorner, gets stored in rpNewFilletEdge->m_cpCorner
                                                    //      NULL to ignore, default:[NULL]
    pFE1->SetFilletEdgeType(SM_FE_CROSS_SECTION);
    SmFilletEdgeuse * pPrimEU1 = (SmFilletEdgeuse*)pFE1->GetPrimaryEdgeuse();
    pPrimEU1->SetFilletGeom(pConvexFG);
    m_vEdges.Add(pFE1);

    SmFilletVertex * pFV3 = new (m_pPseudoBrep) SmFilletVertex(this);
    pFV3->SetPointClass(SM_PC_VERTEX, (SmObject*)m_cpVertex);
    pFV3->SetFilletVertexType(SM_FV_ON_VERTEX);
    m_vVertices.Add(pFV3);

    // Find the original face
    SmFace * pOrigFace = sm_GetFaceByTwoEdgeuses(pCurrEU,pConvexEdgeuse,NULL);
    // Make an extended edge
    SmFilletEdge * pFE2 = NULL;
    SER(SmFilletEdge::MakeFilletEdge(m_pPseudoBrep, // in : target Brep to receive new topology objects
                                     pFV2,          // in : start of new FilletEdge
                                     pFV3,          // in : end   of new FilletEdge
                                     pFE2,          // out: newly allocated FilletEdge - no Curve or UVTrimCurve data
                                     this));        // in : NewFilletEdge's filletCorner, gets stored in rpNewFilletEdge->m_cpCorner
                                                    //      NULL to ignore, default:[NULL]
    pFE2->SetFilletEdgeType(SM_FE_ON_EXTEND_EDGE);
    pFE2->SetOriginalEdge(pCurrEU->GetEdge());
    pFE2->SetOriginalFace(pOrigFace);
    m_vEdges.Add(pFE2);

    // Add the second edge
    SmFilletEdge * pFE3 = NULL;
    SER(SmFilletEdge::MakeFilletEdge(m_pPseudoBrep, // in : target Brep to receive new topology objects
                                     pFV3,          // in : start of new FilletEdge
                                     pFV0,          // in : end   of new FilletEdge
                                     pFE3,          // out: newly allocated FilletEdge - no Curve or UVTrimCurve data
                                     this));        // in : NewFilletEdge's filletCorner, gets stored in rpNewFilletEdge->m_cpCorner
                                                    //      NULL to ignore, default:[NULL]
    pFE3->SetFilletEdgeType(SM_FE_ON_EDGE);
    pFE3->SetOriginalEdge(pCurrEU->GetEdge());
    pFE3->SetOriginalFace(pOrigFace);
    m_vEdges.Add(pFE3);

    // Extend the Side Surface if necessary
    double dExtDist = sm_FindSurfaceExtensionDistAtCorner(
        pConvexEdgeuse,pConvexSolver,m_cpVertex);
    SmFace * pOrigFace1 = sm_GetFaceByTwoEdgeuses(pCurrEU,pConcaveEdgeuse,NULL);
    SmSurface * pSurface = pOrigFace1->GetSurface(); NER(pSurface);
    SmSurface * pNewSurface = NULL;
    SER(pSurface->CreateExtendedSurface(m_crContext,dExtDist,SM_CT_G1,pNewSurface));
    SmObjDelete sCleanSurf(pNewSurface);

    SmSurface *pNewAnalyticSurface;
    // Copy pNewSurface, when possible as an analytic surface
    SER(pNewSurface->CopyAndAddAnalytics(m_crContext,pNewAnalyticSurface));
    AddExtendedOriginalSurface(pOrigFace1,pNewAnalyticSurface);
    pNewAnalyticSurface = NULL;  // to be safe: might have been deleted.

    return SM_SUCCESS;

} // end SmFillet3x2MixedConvexityCorner::MakeCornerTopology

/*******************************************************************//**
PURPOSE: Create a Nx1 Closed corner. Typically, this might happen when we
    have a corner with two or three edges (such as in cylindrical solids)
    where one 'closed' edge is filleted.

NOTES:
***********************************************************************/
SmFilletNx1ClosedCorner::SmFilletNx1ClosedCorner
  (const SmContext      & crContext,
   const SmVertex       * cpVertex,
   SmTArray<SmEdgeuse*> * pSolverEUs,
   SmTArray<SmEdgeuse*> * pAllEUs,
   SmFilletExecutive    * pExec,
   double                 dApproxTol3d,
   double                 dTangencyTolRadians)
 : SmFilletCorner(crContext,cpVertex,pSolverEUs,pAllEUs,pExec,
                  dApproxTol3d,dTangencyTolRadians)
{
    m_bG1Case = FALSE;

} // end SmFilletNx1ClosedCorner::SmFilletNx1ClosedCorner constructor

/*******************************************************************//**
PURPOSE: Creates and connects all the FilletTopology objects needed to
    properly connect an Nx1Closed corner to the filletSurface
    by specifying its boundary FilletEdges and FilletVertices.

    The fact that the filleted Edge is closed means that geometrically, there are
    two filleted Edges at this corner, but they are counted only once since they
    are the same Edge, so we call it Nx1 Closed.  The closed Edge is also counted as 1
    in the count of total edges, so the common case of a single side edge is called
    2x1 Closed.  In the 2x1 Closed case, the topology consists of 2 Fillet Vertices,
    the 1st located on the seamEdge and the 2nd on a face, connected by one fillet edge.
    In the 3x1 Closed case, there are two side edges.  If the two side Edges are on
    opposite sides of the filleted Edge, the topology will depend on the geometry:
    whether or not the rolling ball hits both side Edges at the same location (of the ball).
    That is handled in AdjustTangentCorner(), when we know the geometry.
    In the 3x1 Closed case, we create a FilletVertex on each of the two side Edges.

    When the to-be-filleted edge is G1 closed the shape of the new fillet edge
    is to be computed the same as the crossSection shape of the edge's filletSurface, else the
    shape is to be determined by a fillet/fillet intersection.

    Only defines topology objects and their connections - does not define any
    shape information.

NOTES:
  . Creates 1st FilletVertex (added to m_vVertices list) for each corner edge != filletEdge
    - connects the Fillet Vertex to the railCurve with 2 FilletEdgeuse->FilletVertexuse pairs
    - adds two FilletEdgeuse->FilletVertexuse pair for each filletVertex because the railCurve is closed
  . Creates 2nd FilletVertex (added to m_vVertices list) on the opposing face
    - When FilletEdge is G1 closed - sets VertexType == SM_FV_MATE
      else                                VertexType == SM_FV_FILLET_X_FILLET
    - connects the Fillet Vertex to the railCurve with 2 FilletEdgeuse->FilletVertexuse pairs
    - adds two FilletEdgeuse->FilletVertexuse pair for each filletVertex because the railCurve is closed
  . Creates 1 FilletEdge connecting the 2 new FilletVertices
    - creates 2 pairs of FilletEdgeuse->FilletVertexuse object to connect the new edge
      to the 2 new vertices.
    - When FilletEdge is G1 closed - sets EdgeType == SM_FE_CROSS_SECTION
      else                                EdgeType == SM_FE_FILLET_X_FILLET
    - adds new FilletEdge to corner m_vEdges list.

***********************************************************************/
SmStatus SmFilletNx1ClosedCorner::MakeCornerTopology ()
{
int iDebugLevel = 0;
#ifdef SM_DEBUG_CODE
iDebugLevel = DebugLevel();
#endif // SM_DEBUG_CODE

  // locals - 1st outward bound fillet edgeuse, its FilletSolver, and its 1st fillet geom
  SmEdgeuse      * pFilEdgeuse   = (*m_pSolverEUs)[0];
  SmFilletSolver * pFilSolver    = GetFilletExecutive()->GetFilletSolverOfEdgeuse(pFilEdgeuse);
  SmFilletGeom   * pFilletGeom   = pFilSolver->GetFirstFilletGeom();
  ULONG            lTotalEUs     = m_pAllEUs->GetSize();
  SmEdgeuse      * pSeamEdgeuse  = NULL;

  // Test if the filleted edge is G1-closed
  SmEdge         * pFilletedEdge = pFilEdgeuse->GetEdge();

  m_bG1Case = sm_TestG1Continuity(m_cpVertex,pFilletedEdge,pFilletedEdge,
                                  SM_RAD2DEG(m_dTangencyTolRadians),
                                  NULL, iDebugLevel );

  // for all edges connected to this corner
  for (ULONG ii=0; ii<lTotalEUs; ii++)
    {
      SmEdgeuse * pEU = (*m_pAllEUs)[ii];

      // skip the filletEdge - process the others
      if ( pEU == pFilEdgeuse ) { continue; }

      pSeamEdgeuse = pEU;
      SmPoint3d  sPnt, sPnt2;
      SmVector3d sBinVec, sBinVec2;
      SmVector3d sFaceuseNormal, sFaceuseNormal2;

      // Determine if we need to get its radial EU for
      // SM_FV_RAIL_X_EDGEUSE classification

      // get Seam edgeuse's binormal at vertex (seam is not the fillet edgeuse)
      SmExtent1d sIvl = pSeamEdgeuse->GetEdge()->GetInterval();
      double     dT   = (pSeamEdgeuse->GetOrientation() == SM_OT_SAME)
                        ? sIvl.GetMin()
                        : sIvl.GetMax();
      SER(pSeamEdgeuse->EvaluateBinormal(dT,FALSE,sPnt,sBinVec));

      // get filletEdge's tangent at vertex
      SmVector3d sPV1[2];
      SmExtent1d sIvl1 = pFilEdgeuse->GetEdge()->GetInterval();
      double     dT1   =  (pFilEdgeuse->GetOrientation() == SM_OT_SAME)
                         ? sIvl1.GetMin()
                         : sIvl1.GetMax();
      SER(pFilEdgeuse->GetEdge()->GetCurve()->Evaluate(dT1,1,TRUE,sPV1));

      // when binormal opposes tangent - get radial edgeuse
      if (sPV1[1].Dot(sBinVec) < 0.0)
        { pSeamEdgeuse = pSeamEdgeuse->GetRadial(); }
      NER(pSeamEdgeuse);

      // Create a fillet vertex on the seam edge - label it as intersection of railCurve and edge->Curve
      SmFilletVertex * pFV = new (m_pPseudoBrep) SmFilletVertex(this);
      pFV->SetPointClass(SM_PC_EDGEUSE, pSeamEdgeuse);
      pFV->SetFilletVertexType(SM_FV_RAIL_X_EDGEUSE);
      m_vVertices.Add(pFV);

      // Connect FilletVertex to pFilSolver->pFilletGeom->railCurve that
      //   intersects pSeamEdgeuse with 2 new FilletEdgeuse->FilletVertexuse pairs
      //   -  Need two edgeuse/vertexuse pairs because the rail is closed
      ULONG lRailIndex = pFilSolver->FindIndexOfRailXSideEdgeuse(pSeamEdgeuse);
      SmFilletEdgeuse * pNewEdgeuse = NULL;
      SER(pFilletGeom->MakeRailEdgeuse(m_pPseudoBrep,
          lRailIndex,pFV,pNewEdgeuse,SM_OT_SAME));
      SER(pFilletGeom->MakeRailEdgeuse(m_pPseudoBrep,
          lRailIndex,pFV,pNewEdgeuse,SM_OT_OPPOSITE));

    } // end iter every Edge at this vertex

  // create the filletVertex mate - the filletVertex on the other railCurve
  SmFilletVertex * pFV0 = m_vVertices[0];
  if (lTotalEUs == 2)
    {
      // Create a fillet vertex on the side face
      SmFilletVertex * pFV = new (m_pPseudoBrep) SmFilletVertex(this);
      SmFace * pFace = pFilEdgeuse->GetFace();
      if (pFace == pSeamEdgeuse->GetFace())
        {
          pFace = pFilEdgeuse->GetRadial()->GetFace();
        }

      // remember that this vertex connects to the face
      pFV->SetPointClass(SM_PC_FACE, pFace);

      // for tangent closed-curves - vertex is mate to vertex on seam edge
      if (m_bG1Case)
        {
          // record that the two vertices are mates
          pFV->SetFilletVertexType(SM_FV_MATE);
          pFV->SetMate(0, pFV0);
          pFV0->SetMate(0, pFV);
        }
      else // for non-g1 closed curves - vertex is located at fillet/fillet intersection
        {
          pFV->SetFilletVertexType(SM_FV_FILLET_X_FILLET);
        }

      // add the 2nd vertex to the FilletCorner list
      m_vVertices.Add(pFV);
      ULONG lRailIndex = pFilSolver->FindIndexOfRailOnFace(pFace);

      // connect 2nd vertex to 2nd railCurve with 2 FilletEdgeuse->FilletVertexuse pairs
      // need 2 connections because the railCurve is closed.
      SmFilletEdgeuse * pNewEdgeuse = NULL;
      SER(pFilletGeom->MakeRailEdgeuse(m_pPseudoBrep,
          lRailIndex,pFV,pNewEdgeuse,SM_OT_SAME));
      SER(pFilletGeom->MakeRailEdgeuse(m_pPseudoBrep,
          lRailIndex,pFV,pNewEdgeuse,SM_OT_OPPOSITE));

    } // end lTotalEUs == 2 check

  // This case is now implemented, at least for lTotalUEs == 3.  [B660]
  // It is handled in AdjustTangentCorner(), when we know more about the geometry.
  //   else
  //     { SM_DBG_WARN(_T("Unexpected closed curve corner filleting case\n")) ; }
  //

  // Corner consists of one fillet edge only (joining pFV0 & pFV1)
  SmFilletVertex * pFV1        = m_vVertices[1];
  SmFilletEdge   * pFilletEdge = NULL;
  SM_ASSERT(pFV1 != NULL) ;

  // create a new FilletEdge between the 2 new FilletVertices
  SER(SmFilletEdge::MakeFilletEdge(m_pPseudoBrep,  // in : target Brep to receive new topology objects
                                   pFV0,           // in : start of new FilletEdge
                                   pFV1,           // in : end   of new FilletEdge
                                   pFilletEdge,    // out: newly allocated FilletEdge - no Curve or UVTrimCurve data
                                   this));         // in : NewFilletEdge's filletCorner, gets stored in rpNewFilletEdge->m_cpCorner
                                                   //      NULL to ignore, default:[NULL]
  // when edge is tangent closed - edge shape = filletSurface crossSection shape
  if (m_bG1Case)
    {
      // edge shape is same as filletSurface crossSection shape
      pFilletEdge->SetFilletEdgeType(SM_FE_CROSS_SECTION);
      m_bBlending = TRUE;
    }
  else // else closed curve not tangent - edge Shape = fillet/fillet intersection
    {
      // Split the fillet so that we don't self-intersection
      pFilletGeom->SetSplitFlag(TRUE);
      pFilletEdge->SetFilletEdgeType(SM_FE_FILLET_X_FILLET);
      m_bBlending = FALSE;
    }

  // get edgeuses marking filletEdge sector
  SmTArray<SmEdgeuse*> sFilEdgeuses;
  pFilletEdge->GetEdgeuses(sFilEdgeuses);
  if (sFilEdgeuses.GetSize() != 2) { SER(SM_ERR); }

  // set FilletEdgeuse->FilletGeom pointers
  SmFilletEdgeuse * pPrimEU = (SmFilletEdgeuse*)pFilletEdge->GetPrimaryEdgeuse();
  pPrimEU->SetFilletGeom(pFilletGeom);
  SmFilletEdgeuse * pMateEU = (SmFilletEdgeuse*)pPrimEU->GetMate();
  pMateEU->SetFilletGeom(pFilletGeom);

  // add this edge to the corner list of edges
  m_vEdges.Add(pFilletEdge);

  // all done
  return SM_SUCCESS;

} // end SmFilletNx1ClosedCorner::MakeCornerTopology

/*******************************************************************//**
PURPOSE: If this FilletCorner has exactly two FilletSolvers, and their Edges
   meet with G1 continuity, return the other one than the passed-in argument.

NOTES:
   If the passed-in FilletSolver is not one of ours, return Null with an error.
   If there is only one FilletSolver, and it is the one passed in, return it.

   This class of FilletCorner has only one FilletSolver, not necessarily G1.
***********************************************************************/
SmFilletSolver * SmFilletNx1ClosedCorner::GetTangentFilletSolver( const SmFilletSolver *pFS )
{
  if ( m_pSolverEUs->GetSize() != 1 )  { SM_ASSERT_ERR; return NULL; }

  SmEdgeuse      * pEdgeUse   = (*m_pSolverEUs)[0];
  SmFilletSolver * pFilSolver = m_pExecutive->GetFilletSolverOfEdgeuse( pEdgeUse );

  if ( pFilSolver != pFS )  { SM_ASSERT_ERR; return NULL; }

  // We have to test its continuity.
  SmEdge *pEdge = pEdgeUse->GetEdge();
  int iDebugLevel = 0;  // ... enhance as needed.
  double dTangencyTolDeg = SM_RAD2DEG( m_dTangencyTolRadians );
  if ( sm_TestG1Continuity( m_cpVertex, pEdge, pEdge, dTangencyTolDeg, NULL, iDebugLevel ))
    { return pFilSolver; }

  return NULL; // not an error.

} // end SmFilletNx1ClosedCorner::GetTangentFilletSolver


/*******************************************************************//**
PURPOSE: Create a concave NxN corner (i.e. all the edges are filleted
    which includes at least two concave edges)
    This default corner surface could be a 3-, 4- or N-sided patch which
    blends smoothly between adjacent fillets with one extra boundary edge
    which interpolates two rails from concave edges.

NOTES:
***********************************************************************/
SmFilletConcaveNxNCorner::SmFilletConcaveNxNCorner
  (const SmContext      & crContext,
   const SmVertex       * cpVertex,
   SmTArray<SmEdgeuse*> * pSolverEUs,
   SmTArray<SmEdgeuse*> * pAllEUs,
   SmFilletExecutive    * pExec,
   double                 dApproxTol3d,
   double                 dTangencyTolRadians)
 : SmFilletConvexNxNCorner(crContext,
                           cpVertex,
                           pSolverEUs,
                           pAllEUs,
                           pExec,
                           dApproxTol3d,
                           dTangencyTolRadians)
{

} // end SmFilletConcaveNxNCorner::SmFilletConcaveNxNCorner constructor

/*******************************************************************//**
PURPOSE: Make corner topology which consists of an array of
    SmFilletVertex and an array of SmFilletEdge.

NOTES:
***********************************************************************/
SmStatus SmFilletConcaveNxNCorner::MakeCornerTopology
  ()
{
    // Determine if this is a setback corner
    if (m_dSetBackDist > this->m_dThisApproxTol3d*100.0) {
        // Let SmFilletConvexNxNCorner handle setbacks
        return SmFilletConvexNxNCorner::MakeCornerTopology();
    }

int iDebugLevel = 0;
#ifdef SM_DEBUG_CODE
    iDebugLevel = DebugLevel();
    if ( iDebugLevel > 0 ) {
        for (ULONG f=0; f<m_pAllEUs->GetSize(); f++) {
            smgfx_SetLook(1,2, 1,0,0); (*m_pAllEUs)[f]->Draw(); sm_GraphicsLoop();
        }
        sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE
    // All the edges of corner are filletted
    SmTArray<SmEdgeuse*> sConcaveEUs;
    sm_FindConcaveEdgeuses(m_cpVertex,m_pAllEUs,sConcaveEUs, iDebugLevel );
    if (sConcaveEUs.GetSize() != 2)
        SER(SM_ERR);
    SmEdgeuse * pConcaveEU1 = sConcaveEUs[0];
    SmEdgeuse * pConcaveEU2 = sConcaveEUs[1];
    ULONG lTotalEUs = m_pAllEUs->GetSize();
    ULONG lTotal = lTotalEUs;
    // Allocate 'lTotalEUs' Fillet Vertices
    for (ULONG k=0; k<lTotal; k++) {
        SmFilletVertex * pFV = new (m_pPseudoBrep) SmFilletVertex(this);
        m_vVertices.Add(pFV);
    }
    SmEdgeuse * pPrevEU = (*m_pAllEUs)[lTotalEUs-1];
    SmFilletSolver * pPrevFilSolver = GetFilletExecutive()->
        GetFilletSolverOfEdgeuse(pPrevEU);
    NER(pPrevFilSolver);
    SmFilletVertex * pPrevFV = m_vVertices[lTotal-1];
    ULONG ii, jj;
    for (ii=0, jj=0; ii<lTotalEUs; ii++, jj++) {
        SmEdgeuse * pCurrEU =  (*m_pAllEUs)[ii];
        SmFilletVertex * pCurrFV = m_vVertices[jj];

        SmFace * pCommonFace = sm_GetFaceByTwoEdgeuses(pPrevEU,pCurrEU,NULL);
        NER(pCommonFace);//Unrecoverable error
        pCurrFV->SetPointClass(SM_PC_FACE, pCommonFace);

        SmFilletSolver * pCurrFilSolver = GetFilletExecutive()->
            GetFilletSolverOfEdgeuse(pCurrEU);
        NER(pCurrFilSolver);
        ULONG lRailIndex = pPrevFilSolver->FindIndexOfRailOnFace(pCommonFace);
        ULONG lOtherRailIndex = pCurrFilSolver->FindIndexOfRailOnFace(pCommonFace);
        SmFilletGeom * pPrevFilletGeom = pPrevFilSolver->GetFirstFilletGeom();
        SmFilletGeom * pCurrFilletGeom = pCurrFilSolver->GetFirstFilletGeom();
        if (   (pPrevEU == pConcaveEU1 && pCurrEU == pConcaveEU2) 
            || (pPrevEU == pConcaveEU2 && pCurrEU == pConcaveEU1)) 
          {
            // Process concave corner of pCommonFace
            // Add one more Fillet Point
            SmFilletVertex * pExtraVert = new (m_pPseudoBrep) SmFilletVertex(this);
            m_vVertices.InsertAt(jj++, pExtraVert);
            lTotal++;
            pExtraVert->SetPointClass(SM_PC_FACE, pCommonFace);
            pExtraVert->SetFilletVertexType(SM_FV_MATE);
            SmFilletEdgeuse * pNewEdgeuse = NULL;
            SER(pPrevFilletGeom->MakeRailEdgeuse(m_pPseudoBrep,
                lRailIndex,pExtraVert,pNewEdgeuse));
            pExtraVert->SetMate(0, pPrevFV);
            pPrevFV->SetMate(0, pExtraVert);
            // Make fillet edge
            SmFilletEdge * pFilletEdge = NULL;
            SER(SmFilletEdge::MakeFilletEdge(m_pPseudoBrep,   // in : target Brep to receive new topology objects
                                             pPrevFV,         // in : start of new FilletEdge
                                             pExtraVert,      // in : end   of new FilletEdge
                                             pFilletEdge,     // out: newly allocated FilletEdge - no Curve or UVTrimCurve data
                                             this));          // in : NewFilletEdge's filletCorner, gets stored in rpNewFilletEdge->m_cpCorner
                                                              //      NULL to ignore, default:[NULL]
            pFilletEdge->SetFilletEdgeType(SM_FE_CROSS_SECTION);
            SmFilletEdgeuse * pPrimEU = (SmFilletEdgeuse*)pFilletEdge->
                GetPrimaryEdgeuse();
            pPrimEU->SetFilletGeom(pPrevFilletGeom);
            m_vEdges.Add(pFilletEdge);

            // Setup curr vert
            pCurrFV->SetFilletVertexType(SM_FV_MATE);
            SER(pCurrFilletGeom->MakeRailEdgeuse(m_pPseudoBrep,
                lOtherRailIndex,pCurrFV,pNewEdgeuse));
            SmFilletVertex * pNextFV = m_vVertices[(jj+1)%lTotal];
            pCurrFV->SetMate(0, pNextFV);
            pNextFV->SetMate(0, pCurrFV);
            // Make fillet edge between extra vert and current vert
            pFilletEdge = NULL;
            SER(SmFilletEdge::MakeFilletEdge(m_pPseudoBrep,  // in : target Brep to receive new topology objects
                                             pExtraVert,     // in : start of new FilletEdge
                                             pCurrFV,        // in : end   of new FilletEdge
                                             pFilletEdge,    // out: newly allocated FilletEdge - no Curve or UVTrimCurve data
                                             this));         // in : NewFilletEdge's filletCorner, gets stored in rpNewFilletEdge->m_cpCorner
                                                             //      NULL to ignore, default:[NULL]
            pFilletEdge->SetFilletEdgeType(SM_FE_RAIL_RAIL_INTERPOLATION);
            pFilletEdge->SetOriginalFace(pCommonFace);
            m_vEdges.Add(pFilletEdge);
          }
        else {
            pCurrFV->SetFilletVertexType(SM_FV_RAIL_X_RAIL);
            // Setup rails of fillet
            SmFilletEdgeuse * pNewEdgeuse = NULL;
            SER(pPrevFilletGeom->MakeRailEdgeuse(m_pPseudoBrep,
                lRailIndex,pCurrFV,pNewEdgeuse));
            SER(pCurrFilletGeom->MakeRailEdgeuse(m_pPseudoBrep,
                lOtherRailIndex,pCurrFV,pNewEdgeuse));

            // Make fillet edge
            SmFilletEdge * pFilletEdge = NULL;
            SER(SmFilletEdge::MakeFilletEdge(m_pPseudoBrep, // in : target Brep to receive new topology objects
                                             pPrevFV,       // in : start of new FilletEdge
                                             pCurrFV,       // in : end   of new FilletEdge
                                             pFilletEdge,   // out: newly allocated FilletEdge - no Curve or UVTrimCurve data
                                             this));        // in : NewFilletEdge's filletCorner, gets stored in rpNewFilletEdge->m_cpCorner
                                                            //      NULL to ignore, default:[NULL]
            pFilletEdge->SetFilletEdgeType(SM_FE_CROSS_SECTION);
            SmFilletEdgeuse * pPrimEU = (SmFilletEdgeuse*)pFilletEdge->
                GetPrimaryEdgeuse();
            pPrimEU->SetFilletGeom(pPrevFilletGeom);
            m_vEdges.Add(pFilletEdge);
        }
        pPrevFilSolver = pCurrFilSolver;
        pPrevFV = pCurrFV;
        pPrevEU = pCurrEU;
    }
    m_bBlending = TRUE;

    return SM_SUCCESS;

} // end SmFilletConcaveNxNCorner::MakeCornerTopology


/*******************************************************************//**
PURPOSE:  Test and see if a rail-rail-interpolation curve
    intersected with the original corner

NOTES: Although this is not a common case, sometimes the
    curve may 'cut-through' the corner such as in the 'acute angle' cases
***********************************************************************/
SmBoolean SmFilletConcaveNxNCorner::TestRailRailInterpolationCurve
  (SmCurve * pCurve)
{
    // Check to see if the original corner 'intersects' the curve
    // Drop corner vertex onto pCurve
    SmPoint3d sCornerPnt = m_cpVertex->GetPoint();
    SmExtent1d sIvl = pCurve->GetNaturalInterval();
    SmSolution sSData[8];
    SmSolutionArray sSolutions(8,sSData);
    SER(pCurve->GlobalPointSolve(sIvl,SM_SO_MINIMIZE,sCornerPnt,
        m_dThisApproxTol3d,NULL,NULL,SM_SR_SINGLE,sSolutions));
    if (sSolutions.GetSize() != 1) SER(SM_ERR);
    double dT = sSolutions[0].m_vStart[0];
    // Find the 2nd deriv. of the curve at dT
    SmVector3d sGeomVec[4];
    SER(pCurve->EvaluateGeometric(dT,2,TRUE,sGeomVec));
    SmVector3d sV = sCornerPnt - sGeomVec[0];
    SER(sGeomVec[2].Unitize());
    SER(sV.Unitize());
#ifdef SM_DEBUG_CODE
    if ( DebugLevel() > 0 ) {
        smgfx_SetLook(1,2, 0,1,1); pCurve->Draw(); sm_GraphicsLoop();
        smgfx_SetLook(1,2, 1,0,0); sGeomVec[0].Draw(); sm_GraphicsLoop();
        sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE
    if (sV.Dot(sGeomVec[2]) < 0.0) {
        return TRUE;
    }

    return FALSE;

} // end SmFilletConcaveNxNCorner::TestRailRailInterpolationCurve


/*******************************************************************//**
PURPOSE: Create a convex NxN corner (i.e. all the edges are filleted
    and each of them is a convex edge)
    This default corner surface could be a 3-, 4- or N-sided patch which
    blends smoothly between adjacent fillets

NOTES:
***********************************************************************/
SmFilletConvexNxNCorner::SmFilletConvexNxNCorner
  (const SmContext      & crContext,
   const SmVertex       * cpVertex,
   SmTArray<SmEdgeuse*> * pSolverEUs,
   SmTArray<SmEdgeuse*> * pAllEUs,
   SmFilletExecutive    * pExec,
   double                 dApproxTol3d,
   double                 dTangencyTolRadians)
 : SmFilletNxNCorner(crContext,
                     cpVertex,
                     pSolverEUs,
                     pAllEUs,
                     pExec,
                     dApproxTol3d,
                     dTangencyTolRadians)
{

} // end SmFilletConvexNxNCorner::SmFilletConvexNxNCorner constructor


/*******************************************************************//**
PURPOSE: Adjust the corner topology for the convex cases where a
    (3-sided-) coner surface may be too 'tight'. For example, when filleting
    a box and the radii of three fillets that come to meet at a corner are
    1, 1 and 2. Then the corner patch can be stretched by adding
    another interpolating edge to the patch, thus resulted in 4-sided
    blending surface.

NOTES:
***********************************************************************/
SmStatus SmFilletConvexNxNCorner::AdjustTopology
  ()
{
    // Only non-chamfer & three-sided corners can be adjusted for now
    ULONG lTotal = m_vVertices.GetSize();
    if (lTotal > 3/* || m_bChamfer*/) {
        return SM_SUCCESS;
    }

    ULONG lFoundVertIndex = 9999;
    double dMinDist = SM_BIG_DOUBLE;
    double dMaxDist = 0.0;
    ULONG ii, jj;

    // Look for a vertex of the corner patch that can be 'stretched',
    // i.e. replacing the vertex by a rail-rail-interpolating edge
    for (ii=0; ii<lTotal; ii++) {
        SmFilletVertex * pCurrVert = m_vVertices[ii];
        if (pCurrVert->GetFilletVertexType() != SM_FV_RAIL_X_RAIL) {
            SER(SM_ERR); // Unknown case
        }
        SmFilletVertex * pNextVert = m_vVertices[(ii+1)%lTotal];

        SmPoint3d sVertPnt = pCurrVert->GetPoint();
        double dDistToNextVert = sVertPnt.DistanceBetween(pNextVert->GetPoint());
        if (dMinDist > dDistToNextVert) {
            dMinDist = dDistToNextVert;
        }

        // Get the original face which pCurrVert is on
        SmFace *pOrigFace = SM_CAST_PTR(SmFace,pCurrVert->GetPointClassObject());
        NER(pOrigFace);

        SmBoolean bIsCandidate = TRUE;
        SmTArray<SmVertexuse*> sVertexuses;
        pCurrVert->GetVertexuses(sVertexuses);
        SmPoint3d sPnt[4];
        SmVector3d sVec[2];
        ULONG lCount = 0;
        // Try to determine the estimated distance between two adjacent fillets
        // if iso-edges are used in the stretched corner patch.
        for (jj=0; jj<sVertexuses.GetSize(); jj++) {
            SmFilletEdgeuse * pEU = (SmFilletEdgeuse*)sVertexuses[jj]->GetEdgeuse();
            SmFilletEdge * pE = (SmFilletEdge*)pEU->GetEdge();
            if (pE->GetFilletEdgeType() != SM_FE_CROSS_SECTION) continue;
            SmFilletGeom * pFilletGeom = pEU->GetFilletGeom();
            if (pFilletGeom == NULL) {
                SmFilletEdgeuse * pMateEU = (SmFilletEdgeuse*)pEU->GetMate();
                pFilletGeom = pMateEU->GetFilletGeom();
            }
            NER(pFilletGeom);
            SmFilletSolver * pFilSolver = pFilletGeom->GetFilletSolver();

            SmFilletVertex * pOtherV = (SmFilletVertex*)pE->GetOtherVertex(pCurrVert);
            SmFilletVertexuse * pVU = pOtherV->GetVUAtRailEnd(pFilletGeom);
            SmTsectPnt & rTsectPnt = pVU->GetTsectPnt();
            ULONG lRailIndex = pFilSolver->FindIndexOfRailOnFace(pOrigFace);
            SmPoint2d sUV = rTsectPnt.UVPos(lRailIndex);
            SER(pOrigFace->GetSurface()->EvaluatePoint(sUV,sPnt[lCount]));

            // The following is a simple way to determine if two iso-edges
            // are crossing each other.
//cbi: Fillet reg 1:127: the next vec is essentially zero: what does that mean?
            sVec[lCount] = sPnt[lCount] - sVertPnt;
            SmEdge * pFilletedEdge = pFilSolver->GetEdgeuse(0)->GetEdge();
            SmExtent1d sIvl = pFilletedEdge->GetInterval();
            SmVector3d sPV[2];
            SmCurve * pCurve = pFilletedEdge->GetCurve(); NER(pCurve);
            if (pFilletedEdge->GetStartVertex() == m_cpVertex) {
                SER(pCurve->Evaluate(sIvl.GetMin(),1,TRUE,sPV));
            }
            else {
                SER(pCurve->Evaluate(sIvl.GetMax(),1,TRUE,sPV));
                sPV[1] = -sPV[1];
            }
            if (sPV[1].Dot(sVec[lCount]) < -SM_EFF_ZERO) {
                // Reject this vertex if crossing of the iso-edges happened
                bIsCandidate = FALSE;
                break;
            }
            lCount++;
        }

        if (!bIsCandidate) continue;
#ifdef SM_DEBUG_CODE
        if ( DebugLevel() > 0 ) {
            smgfx_SetLook(1,2, 1,0,0); sVec[0].Draw(&sVertPnt); sm_GraphicsLoop();
            smgfx_SetLook(1,2, 0,1,0); sVec[1].Draw(&sVertPnt); sm_GraphicsLoop();
            smgfx_SetLook(1,2, 0,0,1); sVertPnt.Draw(); sm_GraphicsLoop();
            sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE

        SM_ASSERT(lCount == 2);
        double dVecLen1 = sVec[0].Length();
        double dVecLen2 = sVec[1].Length();
        if (dVecLen1 < 0.2*dVecLen2 || dVecLen2 < 0.2*dVecLen1) {
            continue;
        }

        double dDist = sPnt[0].DistanceBetween(sPnt[1]);
        if (dDist > dMaxDist) {
            dMaxDist = dDist;
            lFoundVertIndex = ii;
        }
    }

    // Determine if we have found a good candidate. Let's use
    // 1/5th of the shortest corner edge as the threshold
    if (lFoundVertIndex == 9999 || dMaxDist < dMinDist/5.0) {
        // Do not adjust this corner
        return SM_SUCCESS;
    }

    // Insert an interpolating edge to substitute the found vertex
    SmFilletVertex * pAdjustVert = m_vVertices[lFoundVertIndex];
    SmFace *pOrigFace = SM_CAST_PTR(SmFace,pAdjustVert->GetPointClassObject());
    SmSurface * pOrigSurf = pOrigFace->GetSurface();
    NER(pOrigSurf);

    // Determine where to inset the new edge
    ULONG lFoundEdgeIndex = 9999;
    SmFilletEdge * pNextE = NULL;
    SmFilletEdge * pPrevE = m_vEdges[lTotal-1];
    for (ii=0; ii<lTotal; ii++) {
        pNextE = m_vEdges[ii];
        if (pNextE->GetVertex() == pAdjustVert) {
            lFoundEdgeIndex = ii;
            break;
        }
        pPrevE = pNextE;
    }
    SM_ASSERT(lFoundEdgeIndex != 9999);

    // Set pAdjustVert as the SM_FV_MATE type and compute its geometry
    SmFilletEdgeuse * pPrevEU = (SmFilletEdgeuse*)pPrevE->GetPrimaryEdgeuse();
    SmFilletGeom * pPrevFG = pPrevEU->GetFilletGeom(); NER(pPrevFG);
    SmFilletVertex * pPrevV = m_vVertices[((lFoundVertIndex+lTotal)-1)%lTotal];
    SmFilletVertexuse * pPrevVU = pPrevV->GetVUAtRailEnd(pPrevFG);
    SmFilletSolver * pPrevFS = pPrevFG->GetFilletSolver();
    ULONG lRailIndex1 = pPrevFS->FindIndexOfRailOnFace(pOrigFace);
    SmTsectPnt & rTsectPnt1 = pPrevVU->GetTsectPnt();
    SmPoint2d sUV1 = rTsectPnt1.UVPos(lRailIndex1);
    SmPoint3d sPnt1;
    SER(pOrigSurf->EvaluatePoint(sUV1,sPnt1));
    pAdjustVert->SetOriginalUV(sUV1);
    pAdjustVert->SetPoint(sPnt1);
    pAdjustVert->SetFilletVertexType(SM_FV_MATE);
    pAdjustVert->SetMate(0,pPrevV);
    pPrevV->SetMate(0,pAdjustVert);
    SmFilletVertexuse * pVU1 = pAdjustVert->GetVUAtRailEnd(pPrevFG);
    NER(pVU1);
    SmTsectPnt & rAdjTsectPnt1 = pVU1->GetTsectPnt();
    rAdjTsectPnt1.UVPos(0) = rTsectPnt1.UVPos(0);
    rAdjTsectPnt1.UVPos(1) = rTsectPnt1.UVPos(1);

    // Make a new vertex
    SmFilletVertex * pNewVert = new (m_pPseudoBrep) SmFilletVertex(this);
    pNewVert->SetPointClass(SM_PC_FACE,pOrigFace);

    // Compute its geometry
    SmFilletEdgeuse * pNextEU = (SmFilletEdgeuse*)pNextE->GetPrimaryEdgeuse();
    SmFilletGeom * pNextFG = pNextEU->GetFilletGeom(); NER(pNextFG);
    SmFilletVertex * pNextV = m_vVertices[(lFoundVertIndex+1)%lTotal];
    SmFilletVertexuse * pNextVU = pNextV->GetVUAtRailEnd(pNextFG);
    SmFilletSolver * pNextFS = pNextFG->GetFilletSolver();
    ULONG lRailIndex2 = pNextFS->FindIndexOfRailOnFace(pOrigFace);
    SmTsectPnt & rTsectPnt2 = pNextVU->GetTsectPnt();
    SmPoint2d sUV2 = rTsectPnt2.UVPos(lRailIndex2);
    SmPoint3d sPnt2;
    SER(pOrigSurf->EvaluatePoint(sUV2,sPnt2));
    pNewVert->SetOriginalUV(sUV2);
    pNewVert->SetPoint(sPnt2);
    pNewVert->SetPointClass(SM_PC_FACE,pOrigFace);
    pNewVert->SetFilletVertexType(SM_FV_MATE);
    pNewVert->SetMate(0,pNextV);
    pNextV->SetMate(0,pNewVert);
    pNewVert->SetStatus(SM_FIL_PROCESSED);
    SmFilletVertexuse * pVU2 = pAdjustVert->GetVUAtRailEnd(pNextFG);
    NER(pVU2);
    SmTsectPnt & rAdjTsectPnt2 = pVU2->GetTsectPnt();
    rAdjTsectPnt2.UVPos(0) = rTsectPnt2.UVPos(0);
    rAdjTsectPnt2.UVPos(1) = rTsectPnt2.UVPos(1);
#ifdef SM_DEBUG_CODE
    if ( DebugLevel() > 0 ) {
        smgfx_SetLook(1,2, 1,0,0); sPnt1.Draw(); sm_GraphicsLoop();
        smgfx_SetLook(1,2, 0,1,0); sPnt2.Draw(); sm_GraphicsLoop();
        sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

    // Move pVU2 from pAdjustVert to pNewVert
    pAdjustVert->Remove(pVU2);
    SER(pNewVert->PostInsert(pVU2));

    SmVertexuse * pVU3 = pNextEU->GetVertexuse();
    if (pVU3->GetVertex() != pAdjustVert) {
        pVU3 = pNextEU->GetMate()->GetVertexuse();
    }
    // Move pVU3 from pAdjustVert to pNewVert
    pAdjustVert->Remove(pVU3);
    SER(pNewVert->PostInsert(pVU3));

    m_vVertices.InsertAt(lFoundVertIndex,pNewVert);

    SmFilletEdge * pNewEdge = NULL;
    SER(SmFilletEdge::MakeFilletEdge(m_pPseudoBrep, // in : target Brep to receive new topology objects
                                     pAdjustVert,   // in : start of new FilletEdge
                                     pNewVert,      // in : end   of new FilletEdge
                                     pNewEdge,      // out: newly allocated FilletEdge - no Curve or UVTrimCurve data
                                     this));        // in : NewFilletEdge's filletCorner, gets stored in rpNewFilletEdge->m_cpCorner
                                                    //      NULL to ignore, default:[NULL]
    pNewEdge->SetFilletEdgeType(SM_FE_RAIL_RAIL_INTERPOLATION);
    pNewEdge->SetOriginalFace(pOrigFace);

    m_vEdges.InsertAt(lFoundEdgeIndex,pNewEdge);

    m_bTopologyAdjusted = TRUE;

    return SM_SUCCESS;

} // end SmFilletConvexNxNCorner::AdjustTopology


/*******************************************************************//**
PURPOSE: Make corner topology which consists of an array of
    SmFilletVertex and an array of SmFilletEdge.

NOTES:
***********************************************************************/
SmStatus SmFilletConvexNxNCorner::MakeCornerTopology
  ()
{
    // Determine if this is a setback corner
    SmBoolean bIsSetBack = FALSE;
    if (m_dSetBackDist > this->m_dThisApproxTol3d*100.0) {
        bIsSetBack = TRUE;
    }

#ifdef SM_DEBUG_CODE
    if ( DebugLevel() > 0 ) {
        for (ULONG f=0; f<m_pAllEUs->GetSize(); f++) {
            smgfx_SetLook(1,2, 1,0,0); (*m_pAllEUs)[f]->Draw(); sm_GraphicsLoop();
        }
        sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE
    // All the edges of corner are filletted
    SmTArray<SmEdge*> sEdges;
    m_cpVertex->GetEdges(sEdges);
    ULONG lTotalEUs = m_pAllEUs->GetSize();

    SmEdgeuse * pPrevEU = (*m_pAllEUs)[(lTotalEUs-1)];
    SmFilletSolver * pPrevFilSolver = GetFilletExecutive()->
        GetFilletSolverOfEdgeuse(pPrevEU);
    NER(pPrevFilSolver);
    SmFilletSolver * pLastFilSolver = pPrevFilSolver;
    // Create all fillet vertices
    ULONG ii;
    for (ii=0; ii<lTotalEUs; ii++) {
        SmEdgeuse * pCurrEU =  (*m_pAllEUs)[ii];
        SmFilletSolver * pCurrFilSolver = GetFilletExecutive()->
            GetFilletSolverOfEdgeuse(pCurrEU);
        if (pCurrFilSolver == NULL) {
            pCurrFilSolver = GetFilletExecutive()->
                GetFilletSolverOfEdgeuse(pCurrEU->GetMate());
            NER(pCurrFilSolver);
        }
        SmFace * pCommonFace = sm_GetFaceByTwoEdgeuses(
            pPrevEU,pCurrEU,NULL);
        NER(pCommonFace);//Unrecoverable error
        ULONG lPrevRailIndex = pPrevFilSolver->FindIndexOfRailOnFace(pCommonFace);
        ULONG lCurrRailIndex = pCurrFilSolver->FindIndexOfRailOnFace(pCommonFace);

        SmFilletVertex * pCurrFV = new (m_pPseudoBrep) SmFilletVertex(this);
        pCurrFV->SetPointClass(SM_PC_FACE, pCommonFace);

        SmFilletGeom * pPrevFilletGeom = pPrevFilSolver->GetFirstFilletGeom();
        SmFilletGeom * pCurrFilletGeom = pCurrFilSolver->GetFirstFilletGeom();
        if (!bIsSetBack) { // Regular corner, no setbacks
            pCurrFV->SetFilletVertexType(SM_FV_RAIL_X_RAIL);
            // Setup rails of fillets
            SmFilletEdgeuse * pNewEdgeuse = NULL;
            SER(pPrevFilletGeom->MakeRailEdgeuse(m_pPseudoBrep,
                lPrevRailIndex,pCurrFV,pNewEdgeuse));
            SER(pCurrFilletGeom->MakeRailEdgeuse(m_pPseudoBrep,
                lCurrRailIndex,pCurrFV,pNewEdgeuse));
            m_vVertices.Add(pCurrFV);
        }
        else {
            // Will insert a new vertex before curr vert
            SmFilletVertex * pNewV = new (m_pPseudoBrep) SmFilletVertex(this);
            pNewV->SetPointClass(SM_PC_FACE, pCommonFace);
            pNewV->SetFilletVertexType(SM_FV_MATE);
            m_vVertices.Add(pNewV);
            m_vVertices.Add(pCurrFV);
            // Setup rails of fillets
            SmFilletEdgeuse * pNewEdgeuse = NULL;
            SER(pPrevFilletGeom->MakeRailEdgeuse(m_pPseudoBrep,
                lPrevRailIndex,pNewV,pNewEdgeuse));
            pCurrFV->SetFilletVertexType(SM_FV_SETBACK);
            SER(pCurrFilletGeom->MakeRailEdgeuse(m_pPseudoBrep,
                lCurrRailIndex,pCurrFV,pNewEdgeuse));
        }
        pPrevFilSolver = pCurrFilSolver;
        pPrevEU = pCurrEU;
    }
    // Define center vert for Bevel cases
    SmFilletVertex * pCenterFV = NULL;
    if (m_bBevel) {
        m_bBlending = FALSE;
        if (lTotalEUs != 3)
            SER(SM_ERR); // Will not handle other cases for now
        pCenterFV = new (m_pPseudoBrep) SmFilletVertex(this);
        pCenterFV->SetFilletVertexType(SM_FV_FILLET_X2_FILLETS);
        //pCenterFV->SetPoint(m_cpVertex->GetPoint());// Needed for 2-pts intersector
    }
    else {
        // This is a blending corner (it blends all edge-fillets)
        m_bBlending = TRUE;
    }

    // Now, create all fillet edges
    ULONG lTotal = m_vVertices.GetSize();
    pPrevFilSolver = pLastFilSolver;
    ULONG lCount = 0;
    SmFilletVertex * pCurrFV = m_vVertices[lCount];
    for (ii=0; ii<lTotalEUs; ii++) {
        SmEdgeuse * pCurrEU =  (*m_pAllEUs)[ii];
        SmFilletSolver * pCurrFilSolver = GetFilletExecutive()->
            GetFilletSolverOfEdgeuse(pCurrEU);
        if (pCurrFilSolver == NULL) {
            pCurrFilSolver = GetFilletExecutive()->
                GetFilletSolverOfEdgeuse(pCurrEU->GetMate());
            NER(pCurrFilSolver);
        }
        SmFilletGeom * pPrevFilletGeom = pPrevFilSolver->GetFirstFilletGeom();
        SmFilletGeom * pCurrFilletGeom = pCurrFilSolver->GetFirstFilletGeom();
        // Get next vertex
        lCount = (lCount+1)%lTotal;
        SmFilletVertex * pNextFV = m_vVertices[lCount];
        if (bIsSetBack) {
            // Insert a rail-rail-interpo edge
            SmFilletEdge * pE = NULL;
            SER(SmFilletEdge::MakeFilletEdge(m_pPseudoBrep, // in : target Brep to receive new topology objects
                                             pCurrFV,       // in : start of new FilletEdge
                                             pNextFV,       // in : end   of new FilletEdge
                                             pE,            // out: newly allocated FilletEdge - no Curve or UVTrimCurve data
                                             this));        // in : NewFilletEdge's filletCorner, gets stored in rpNewFilletEdge->m_cpCorner
                                                            //      NULL to ignore, default:[NULL]
            pE->SetFilletEdgeType(SM_FE_RAIL_RAIL_INTERPOLATION);
            SmFace *pCommonFace = SM_CAST_PTR(SmFace,
                pNextFV->GetPointClassObject()); NER(pCommonFace);
            pE->SetOriginalFace(pCommonFace);
            m_vEdges.Add(pE);
            pCurrFV = pNextFV;
            lCount = (lCount+1)%lTotal;
            pNextFV = m_vVertices[lCount];
        }
        SmFilletEdge * pFilletEdge = NULL;
        if (m_bBevel) {
            // Make edge from curr vert to center
            SER(SmFilletEdge::MakeFilletEdge(m_pPseudoBrep,   // in : target Brep to receive new topology objects
                                             pCurrFV,         // in : start of new FilletEdge
                                             pCenterFV,       // in : end   of new FilletEdge
                                             pFilletEdge,     // out: newly allocated FilletEdge - no Curve or UVTrimCurve data
                                             this));          // in : NewFilletEdge's filletCorner, gets stored in rpNewFilletEdge->m_cpCorner
                                                              //      NULL to ignore, default:[NULL]
            pFilletEdge->SetFilletEdgeType(SM_FE_FILLET_X_FILLET);
            SmFilletEdgeuse * pPrimEU = (SmFilletEdgeuse*)pFilletEdge->
                GetPrimaryEdgeuse();
            SmFilletEdgeuse * pMateEU = (SmFilletEdgeuse*)pPrimEU->GetMate();
            pPrimEU->SetFilletGeom(pPrevFilletGeom);
            pMateEU->SetFilletGeom(pCurrFilletGeom);
        }
        else {
            // Make a cross-section (iso-curve) on fillet
            SER(SmFilletEdge::MakeFilletEdge(m_pPseudoBrep, // in : target Brep to receive new topology objects
                                             pCurrFV,       // in : start of new FilletEdge
                                             pNextFV,       // in : end   of new FilletEdge
                                             pFilletEdge,   // out: newly allocated FilletEdge - no Curve or UVTrimCurve data
                                             this));        // in : NewFilletEdge's filletCorner, gets stored in rpNewFilletEdge->m_cpCorner
                                                            //      NULL to ignore, default:[NULL]
            pFilletEdge->SetFilletEdgeType(SM_FE_CROSS_SECTION);
            SmFilletEdgeuse * pPrimEU = (SmFilletEdgeuse*)pFilletEdge->
                GetPrimaryEdgeuse();
            pPrimEU->SetFilletGeom(pCurrFilletGeom);
            if (bIsSetBack) {
                pCurrFV->SetMate(0,pNextFV);
                pNextFV->SetMate(0,pCurrFV);
            }
        }
        m_vEdges.Add(pFilletEdge);

        pCurrFV = pNextFV;
        pPrevFilSolver = pCurrFilSolver;
    }

    if (pCenterFV)  // Add center vertex for BEVEL case
        m_vVertices.Add(pCenterFV);

    return SM_SUCCESS;

} // end SmFilletConvexNxNCorner::MakeCornerTopology




/*******************************************************************//**
PURPOSE: Create Nx1 corner where only one edge was filleted.

NOTES:
***********************************************************************/
SmFilletNx1Corner::SmFilletNx1Corner
  (const SmContext      & crContext,
   const SmVertex       * cpVertex,
   SmTArray<SmEdgeuse*> * pSolverEUs,
   SmTArray<SmEdgeuse*> * pAllEUs,
   SmFilletExecutive    * pExec,
   double                 dApproxTol3d,
   double                 dTangencyTolRadians)
 : SmFilletCorner(crContext,
                  cpVertex,
                  pSolverEUs,
                  pAllEUs,
                  pExec,
                  dApproxTol3d,
                  dTangencyTolRadians)
{

} // end SmFilletNx1Corner::SmFilletNx1Corner constructor

/*******************************************************************//**
PURPOSE: For Nx1 cases, if two face-normals at the corner are parallel,
    we'll classify it as degenerate corner

NOTES:
***********************************************************************/
SmBoolean SmFilletNx1Corner::CheckNx1Degeneracy
  (const SmVertex * pCornerVert,
   SmEdgeuse * pFilEdgeuse)
{
    // Get adjacent edgeuses of filleted edgeuse
    SmEdgeuse * pSideEU1 = pFilEdgeuse->GetCWEdgeuse();
    SmEdgeuse * pSideEU2 = pFilEdgeuse->GetRadial()->GetCCWEdgeuse();
    if (pSideEU1 == pSideEU2) {
        // likely 2x1 case
        pSideEU2 = pSideEU1->GetMate();
    }

    SmVertex * pVert1 = pSideEU1->GetVertexuse()->GetVertex();
    SmExtent1d sIvl1 = pSideEU1->GetEdge()->GetInterval();
    double dT1 = sIvl1.GetMax();
    if (   (pVert1 == pCornerVert && pSideEU1->GetOrientation() == SM_OT_SAME)
        || (pVert1 != pCornerVert && pSideEU1->GetOrientation() == SM_OT_OPPOSITE)) 
      {
        dT1 = sIvl1.GetMin();
      }
    SmVertex * pVert2 = pSideEU2->GetVertexuse()->GetVertex();
    SmExtent1d sIvl2 = pSideEU2->GetEdge()->GetInterval();
    double dT2 = sIvl2.GetMax();
    if (   (pVert2 == pCornerVert && pSideEU2->GetOrientation() == SM_OT_SAME)
        || (pVert2 != pCornerVert && pSideEU2->GetOrientation() == SM_OT_OPPOSITE)) 
      {
        dT2 = sIvl2.GetMin();
      }
    SmVector3d sBinPnt1, sBinPnt2, sBinVec1, sBinVec2, sFaceuseNormal1, sFaceuseNormal2;
    SER(pSideEU1->EvaluateBinormal(dT1,FALSE,sBinPnt1,sBinVec1,NULL,&sFaceuseNormal1));
    SER(pSideEU2->EvaluateBinormal(dT2,FALSE,sBinPnt2,sBinVec2,NULL,&sFaceuseNormal2));
    // Check if faceuse normals are parallel to each other
#ifdef SM_DEBUG_CODE
    int iDebugLevel = 0; // Note: can't call DebugLevel() from a static method.
    if ( iDebugLevel > 0 ) {
        smgfx_SetLook(4,10, 1,0,0); pSideEU1->Draw(); sm_GraphicsLoop();
        smgfx_SetLook(4,10, 0,0,1); sFaceuseNormal1.Draw(&sBinPnt1); sm_GraphicsLoop();
        smgfx_SetLook(4,10, 0,1,0); pSideEU2->Draw(); sm_GraphicsLoop();
        sFaceuseNormal2.Draw(&sBinPnt2); sm_GraphicsLoop();
        sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE
    if (sFaceuseNormal1.IsParallelTo(sFaceuseNormal2,SM_EFF_ZERO_SQRT)) {
        return TRUE;
    }
    return FALSE;

} // end SmFilletNx1Corner::CheckNx1Degeneracy

/*******************************************************************//**
PURPOSE: Create 'end' patch from extended 'side-face' of original brep
    if needed and create Face (and Edges and Verts) in m_pFilletBrep 
    from m_vSurfaces with call 
      SmFilletCorner::CreateCornerPatch() or
      SmFilletCorner::CreateEndPatch()

NOTES:
***********************************************************************/
SmStatus SmFilletNx1Corner::CreateCornerPatch
  ()
{
  if (m_vEdges.GetSize() == 3)
    {
      if(m_bExtendedSurfacePatch)
           { SER(CreateEndPatch()); }
      else
           { SER(SmFilletCorner::CreateCornerPatch()); }
    }
  else
    { SM_DBG_WARN(_T("Unexpected SmFilletNx1Corner endFace construction case")) ; }

  return SM_SUCCESS;

} // end SmFilletNx1Corner::CreateCornerPatch

/*******************************************************************//**
PURPOSE: Create Nx1 corner by specifying the FilletEdge
    boundary of the incoming fillet. When needed define
    endFaces and extra FilletEdges as needed.

NOTES:
 For all cases:
 -  new SmFilletVertex objects are constructed and classified to each rail
        m_pTargetBrep->edgeuse coming into this corner.
 -  new SmFilletVertices are added to this SmFilletCorner::m_vVertices array.

 - 1 new SmFilletEdge and 2 new SmFilletEdgeuses are constructed to
          connect appropriate new SmFilletVertex pairs.
 - New SmFilletEdges are added to this SmFilletCorner::m_vEdges array.

 Every Nx1 fillet needs an end Face bounded by a FilletEdge (connecting
 the two side edges at the rail/sideEdge intersection points)
 and the side edges themselves from the original vertex to the
 to the rail/sideEdge intersections.  In various cases
 different bits of the endFace (surface, edges, and vertices) need
 to be defined or are already defined.  Every case
 needs 1 filletEdge defined as the intersection of the
 filletSurface and the endFace.  This edge is always terminated
 by a pair of filletVertices located at rail/sideEdge intersections.

 In addition to this vertex/edge/vertex set, sometimes a face will need
 to be extended (or a new end-cap face defined) because the fillet
 surface would not intersect anything even when extended.  Sometimes,
 in addition to the extended end face, one (but never both) of the
 side edges must be extended in order to intersect the corresponding
 rail edge of the fillet surface.  The cases depend on the relative
 convexities of the fillet edge and two topologically adjacent side
 edges.  (Note, if there are more than three edges, the convexities
 of the remaining 'remote' edges don't matter.)  If all three edges
 have the same convexity, then no extending needs to be done.  In
 mixed-convexity cases, it depends on whether one or both of the side
 edges have convexities opposite to the filleted edge:

 Convexity of side edges relative to the filleted edge:
 ----------
  both same: nothing -- uniform convexity case.
  both opp : extend end face(s).
  same/opp : extend end face(s), and extend the side edge w/ same cvxty.

 (Note, 'side faces' are the two faces that contain the filleted edge,
 and 'end face(s)' are all other faces around this vertex (just one, in
 the normal 3x1 case).  End face(s) are what the fillet surface will
 intersect.)

 In the mixed-convexity case, the end face of the fillet will be either
 an extension of the original end surface, if the new fillet edges from
 the original vertex on the filleted edge to the rail/sideEdge intersections
 lie in the original end surface, otherwise a new end patch will be
 constructed that is G1 with end faces(s) at the two side edges.

 Note, regardless of convexities, there are cases where in which the
 expected intersections of fillet rail curves with side edges, or of
 the fillet surface with end surface(s), will not occur, due to
 geometric problems.  These will happen if the existing geometry
 (side edges or end faces) are short (generally, shorter than the
 fillet radius), or bend away from the vertex too quickly.
 These cases are not (yet) handled by this routine.


***********************************************************************/
SmStatus SmFilletNx1Corner::MakeCornerTopology()
{
  // locals
  SmEdgeuse      * pFilEdgeuse = (*m_pSolverEUs)[0];
  SmFilletSolver * pFilSolver  = GetFilletExecutive()->GetFilletSolverOfEdgeuse(pFilEdgeuse);
  NER(pFilSolver);
  SmFilletGeom   * pFilletGeom = pFilSolver->GetFirstFilletGeom();
  ULONG            lTotalEUs   = m_pAllEUs->GetSize();
  ULONG            lFilEUIndex = SM_BIG_ULONG;

  // get the FIlEdgeuse index in the m_pAllEUs array
  ULONG ii;
  for ( ii=0; ii<lTotalEUs; ii++ )
    {
      if ( (*m_pAllEUs)[ii] == pFilEdgeuse )
        {
          lFilEUIndex = ii;
          break;
        }
    }
  SM_ASSERT(lFilEUIndex != SM_BIG_ULONG);

  // Get Adjacent edge->edgeuses of filleted edge
  // (the edges sharing a face with this filleted edge).

  SmEdgeuse * pEU[2];
  ULONG lIndexA = ((lFilEUIndex+lTotalEUs)-1)%lTotalEUs ; // 'behind' fil index
  ULONG lIndex1 = (lFilEUIndex+1)%lTotalEUs ;           // 'ahead of' fil index
  SM_ASSERT((lIndexA + 1)%lTotalEUs         == lFilEUIndex) ;
  SM_ASSERT(((lIndex1+lTotalEUs)-1)%lTotalEUs == lFilEUIndex) ;
  pEU[0] = (*m_pAllEUs)[lIndexA];
  pEU[1] = (*m_pAllEUs)[lIndex1];

  // check for 2x1 on non-closed-edge case with closed but not periodic side edge.
  SmBoolean bClosedSideEdge = (pEU[1] == pEU[0]) ;

int iDebugLevel = 0;

#ifdef SM_DEBUG_CODE
  // draw FilEdgeuse (red) and neighbor edgeuses (green, yellow)
  // connected to edges sharing a common face
  iDebugLevel = DebugLevel();
  if ( iDebugLevel > 0 )
    {
      smgfx_SetLook(3,5, 1,0,0); pFilEdgeuse->Draw(); sm_GraphicsLoop();
      smgfx_SetLook(3,5, 0,1,0); pEU[0]->Draw();      sm_GraphicsLoop(); // adjacent edgeuses to filleted edge
      smgfx_SetLook(3,5, 1,1,0); pEU[1]->Draw();      sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // Check the relative convexities of the filleted edge
  // and the two topologically adjacent side edges.  (See notes above.)

  SmBoolean bSideEdgeSameCvxty[2];
  bSideEdgeSameCvxty[0] = same_convexity_edges(pFilEdgeuse, pEU[0], m_cpVertex, iDebugLevel );
  bSideEdgeSameCvxty[1] = same_convexity_edges(pFilEdgeuse, pEU[1], m_cpVertex, iDebugLevel );

  SmBoolean bMixedCvxty = ! bSideEdgeSameCvxty[0] || ! bSideEdgeSameCvxty[1];


  // for both rails, create filletvertices for rail/filletEdge intersections
  SmFilletVertex * pFVArray[2];

  // when working with mixed convexity:
  SmEdge * pExtendEdge = NULL; // sideEdge to be extended 
  SmFace * pEndFace    = NULL; // Face to extend when endFace->surface can
                               //   be defined by a surface extension.
                               //   In the 3x1 case, this is a unique face,
                               //   the one between the two side EU's.

  for(ULONG lThisRailIdx=0; lThisRailIdx<2; lThisRailIdx++)
    {
      ULONG lOtherRailIdx = 1 - lThisRailIdx;

      // construct FilletVertex to set on rail, and add it to m_vVertices
      pFVArray[ lThisRailIdx ] = new (m_pPseudoBrep) SmFilletVertex( this );
      m_vVertices.Add( pFVArray[ lThisRailIdx ] );
      SmEdgeuse * pSideEU = NULL;

      // get m_pTargetBrep->edge->edgeUse that intersects this rail
      ULONG lSolverRailIndex;
      if ( bClosedSideEdge )
        {
          // Assign rail index first, then find side edgeuse
          lSolverRailIndex = lThisRailIdx;
          pSideEU = sm_FindClosedSideEdgeuse(
                          lSolverRailIndex, pFilSolver, pEU[ lThisRailIdx ], iDebugLevel );
        }
      else
        { // Non-closed side edge (i.e. regular cases)
          pSideEU = sm_FindSideEdgeuse( pFilEdgeuse, pEU[ lThisRailIdx ], iDebugLevel  );
          lSolverRailIndex = pFilSolver->FindIndexOfRailXSideEdgeuse( pSideEU );
        }

      // set filletVertex to classify to the m_pTargetBrep->sideEdgeuse
      pFVArray[ lThisRailIdx ]->SetPointClass( SM_PC_EDGEUSE, pSideEU );

      // Connect vertex to rail edge with connection sequence:
      //   RailFilletEdge->newFilletEdgeuse->newFilletVertexuse->FilletVertex
      SmFilletEdgeuse * pNewEdgeuse = NULL;

      SER( pFilletGeom->MakeRailEdgeuse( m_pPseudoBrep,
              lSolverRailIndex, pFVArray[ lThisRailIdx ], pNewEdgeuse ));

      // set FilletVertex type:
      //    FilletVertexPosition will be the intersection of the rail
      //    with the side edge.  Note, in the mixed-convexity case,
      //    if the convexity of the side edge is the same as that of
      //    the filleted edge (so the other side edge has opposite cvxty
      //    to both), then this side edge will have to be extended.

      SmFace * pFace = pEU[ lThisRailIdx ]->GetFace();

      // At a mixed-convexity corner, if this side-edge has the same convexity
      // as the filleted edge and the other side-edge has opposite convexity,
      // then we'll need to extend this side-edge ('around the corner' of the
      // other side-edge) so that it will intersect this rail.

      if ( bMixedCvxty  &&  bSideEdgeSameCvxty[ lThisRailIdx ] )
        {
          pFVArray[ lThisRailIdx ]->SetFilletVertexType(SM_FV_RAIL_X_EXTENDED_EDGEUSE);

          // Use the next edgeuse around the vertex, past the current one
          //SmEdgeuse * pExtendEdgeuse =
          //    ( lThisRailIdx==1 )
          //        ? m_pAllEUs->GetAt( ((lFilEUIndex+lTotalEUs)-2)%lTotalEUs)
          //        : m_pAllEUs->GetAt( (lFilEUIndex+2)%lTotalEUs) ;
          //
          // No, extend the edge with the same convexity, because it shares
          // a face with the fillet rail-to-be which it will intersect.
          // [bd, 3 Feb 06]
          SmEdgeuse * pExtendEdgeuse = pEU[ lThisRailIdx ];


          // Find original EndFace to be extended:
          //   the EndFace between the extended edge and the other side edge.
          pEndFace = sm_GetFaceByTwoEdgeuses(
                          pExtendEdgeuse, pEU[ lOtherRailIdx ], NULL );
          pExtendEdge = pExtendEdgeuse->GetEdge();

          // Point Edgeuse to the Fillet (side) Face containing the cliff edge.
          // (i.e. the one that's *not* getting extended.)
          // (i.e. the one into which the edge will be extended.)
          if (   pFace != pFilEdgeuse->GetFace()
              && pFace != pFilEdgeuse->GetRadial()->GetFace())
            {
              pEU[ lThisRailIdx ] = pEU[ lThisRailIdx ]->GetRadial();
            }

#ifdef SM_DEBUG_CODE
          if ( DebugLevel() > 0 ) 
            {
              smgfx_SetLook(1,2, 0,1,0); pExtendEdgeuse->Draw(); sm_GraphicsLoop();
              smgfx_SetLook(1,2, 1,0,0); pEU[lThisRailIdx]->Draw(); sm_GraphicsLoop();
              sm_GraphicsLoop();
            }
#endif // SM_DEBUG_CODE

        } // end extend-side-edge branch
      else  // Regular case
        {
          pFVArray[ lThisRailIdx ]->SetFilletVertexType( SM_FV_RAIL_X_EDGEUSE );

          // Point Edgeuse to an EndFace (not a fillet side face).
          if (   pFace == pFilEdgeuse->GetFace()
              || pFace == pFilEdgeuse->GetRadial()->GetFace())
            {
              pEU[ lThisRailIdx ] = pEU[ lThisRailIdx ]->GetRadial();
            }
        }
    } // end iter both rail vertices


  // What's left to do: create the topology for the end rib curve
  // (intersection of fillet surface with the end face),
  // and, if at a mixed-convexity corner, create a capping surface
  // (or extend the end surface).
  // But, if the end rib is not a simple one-piece thing (i.e.,
  // more than one end face), don't do it here.
  // That would be the case if lTotalEUs > 3.

  if ( !bMixedCvxty && lTotalEUs > 3 )
    {
      // Simple convex corner, OR more than one end face,
      // just let global merge do the work.

      m_pExecutive->SetDoGlobalMerge( TRUE );
      return SM_SUCCESS;
    }


  // Ok, now we're either capping (mixed convexity),
  // or the simple case of <=3 total edges at this vertex.

  // compute how much to extend the endSurface to make sure it covers the 'hole'
  double dExtDist =   bMixedCvxty 
                    ? sm_FindSurfaceExtensionDistAtCorner(pFilEdgeuse, 
                                                          pFilSolver, 
                                                          m_cpVertex )
                    : 0.0;

  // Ok, get to work on the topology of the end of the fillet.
  // Allocate the FilletEdge and its edgeuses connecting the two
  // rail/filletEdge intersection vertices.  This edge will be
  // the intersection of the fillet face with the end face.
  //  - Add new FilletEdge to this SmFilletCorner::m_vEdges list.
  //  - The shape of the new FilletEdge is not yet set - just the
  //    topological connection.

  SmFilletEdge * pFilletEndRibEdge = NULL;
  SER( SmFilletEdge::MakeFilletEdge(m_pPseudoBrep,     // in : target Brep to receive new topology objects
                                    m_vVertices[0],    // in : start of new FilletEdge
                                    m_vVertices[1],    // in : end   of new FilletEdge
                                    pFilletEndRibEdge, // out: newly allocated FilletEdge - no Curve or UVTrimCurve data
                                    this ));           // in : NewFilletEdge's filletCorner, gets stored in rpNewFilletEdge->m_cpCorner
                                                       //      NULL to ignore, default:[NULL]
  pFilletEndRibEdge->SetFilletEdgeType( SM_FE_FILLET_X_SIDE_FACE );
  SmFilletEdgeuse * pPrimEU = (SmFilletEdgeuse*)pFilletEndRibEdge->GetPrimaryEdgeuse();
  pPrimEU->SetFilletGeom( pFilletGeom );
  m_vEdges.Add( pFilletEndRibEdge );

  // In the mixed-cvxty case, pEndFace will be NULL when we arrive here
  // if both side edges have opposite convexity to the filleted edge.
  // In that case, the face connecting the two side edges will need to be
  // used (or extended) to generate an endFace.

  SmBoolean   bMultiNeighborEndFace = FALSE ;
  SmEdgeuse * pOtherSideEdgeuse     = NULL ;

  if ( pEndFace == NULL )
    {
      pEndFace = sm_GetFaceByTwoEdgeuses(pEU[0],pEU[1],NULL);

      // when sm_GetFaceByTwoEdgeuses() fails to find a result
      // there are more than 3 edges in this corner and the neighbor
      // edges are not connected by a single face but rather a sequence
      // of faces.

      if ( pEndFace == NULL )
        {
          // Also note that, from the previous bailout (where we "just let
          // global merge do the work"), we must be in the mixed-convexity
          // case here.
          SM_ASSERT( bMixedCvxty );

          bMultiNeighborEndFace = TRUE ;

          // assume that the endCap can be created by extending the
          // face->Surface connected to the 1st cliff side edge.
          // gwc:TODO in the future, new Surfaces can be defined which are
          //   tangent continuous with both side edges.
          if ( lTotalEUs > 3 )
            {
              MSG(_T("Building a FilletEndFace between two different faces - may be a G0 endPatch"));
              // Use the side edge that is a cliff edge.
              // If the two side edges have opposite convexities to each other,
              // then the cliff edge will be the one with the same convexity as
              // the filleted edge; otherwise (side edges both have opposite
              // convexity to the filleted edge -- remember, we must be in the
              // mixed-convexity case here) they're both cliff edges.
              if ( bSideEdgeSameCvxty[0] )
                {
                  pEndFace          = pEU[0]->GetFace();
                  pExtendEdge       = pEU[0]->GetEdge();
                  pOtherSideEdgeuse = pEU[1] ;
                }
              else if ( bSideEdgeSameCvxty[1] )
                {
                  pEndFace          = pEU[1]->GetFace();
                  pExtendEdge       = pEU[1]->GetEdge();
                  pOtherSideEdgeuse = pEU[0] ;
                }
              else  // both are cliff edges, just pick one.
                {
                  pEndFace          = pEU[0]->GetFace();
                  pExtendEdge       = pEU[0]->GetEdge();
                  pOtherSideEdgeuse = pEU[1] ;
                }
            }
          else
            {
              SM_DBG_WARN(_T("Found an unexpected multiNeighborEndFace with less than 4 vertex->edges")) ;
              SER(SM_ERR); // Invalid brep topology
            }
        }
    } // end pEndFace == NULL check

  pFilletEndRibEdge->SetOriginalFace( pEndFace );

  // compute how much to extend the sideSurface to make sure it covers the 'hole'
  SmBoolean bDoExtend = ( pExtendEdge != NULL );
  if ( dExtDist > SM_EFF_ZERO )
    {
      if (   bDoExtend == FALSE
          && (   pFilSolver->GetSolverType() == SM_FS_CONST_RADIUS
              || pFilSolver->GetSolverType() == SM_FS_CONST_RADIUS_ASSISTED))
        {
          dExtDist *= 0.1;   // use only a small extension
          bDoExtend = TRUE;  // for constant radius only
        }
    }

  // When a surface extension is needed to define the endFace shape,
  // build and save it.

  SmSurface *pNewExtendSurface = NULL ;
  if ( bDoExtend )
    {
      SmSurface * pSurface = pEndFace->GetSurface();
      NER(pSurface);
      SmSurface * pNewSurface = NULL;

      // create an extended surface large enough to fill in the endCap requirements
      SER( pSurface->CreateExtendedSurface(m_crContext, dExtDist, SM_CT_G1, pNewSurface ));
      SmObjDelete sCleanSurf( pNewSurface );

      // Copy pNewSurface, when possible as an analytic surface
      SER( pNewSurface->CopyAndAddAnalytics( m_crContext, pNewExtendSurface ));

      // register the OrigFace/ExtendedSurface pair
      // in the SmFilletExecutive::m_vExtendedSurfacesMap list
      AddExtendedOriginalSurface( pEndFace, pNewExtendSurface );

      // If AddExtendedOriginalSurface() deleted pNewExtendSurface, then
      // pEndFace already had an extended surface, and we don't have to
      // deal with it anymore.  But if pNewExtendSurface stayed in the
      // map, we will have to do further checks on it.  So check whether
      // it changed:
      SmSurface *pTmpSrfPtr = GetExtendedSurface( pEndFace );
      if ( pTmpSrfPtr != pNewExtendSurface )
        {
          pNewExtendSurface = NULL;  // deleted in AddExtendedOriginalSurface()
        }
    }

  // When adding an endSurface to fill the 'hole',
  // set up the topology for the end cap.
  if ( bMixedCvxty )
    {
      // add a filletVertex at the origVertex location
      SmFilletVertex * pFV = new (m_pPseudoBrep) SmFilletVertex(this);
      pFV->SetPointClass(SM_PC_VERTEX, (SmObject*)m_cpVertex);
      pFV->SetFilletVertexType( SM_FV_ON_VERTEX );
      m_vVertices.Add( pFV );

      // add two filletEdges from origVertex to rail/edge intersections

      // Add 1st fillet edge from expected rail2/sideEdge2 intersection to origVertex
      SmFilletEdge * pNewFilletEdge2 = NULL;
      SER(SmFilletEdge::MakeFilletEdge(m_pPseudoBrep,   // in : target Brep to receive new topology objects
                                       m_vVertices[1],  // in : start of new FilletEdge
                                       m_vVertices[2],  // in : end   of new FilletEdge
                                       pNewFilletEdge2, // out: newly allocated FilletEdge - no Curve or UVTrimCurve data
                                       this));          // in : NewFilletEdge's filletCorner, gets stored in rpNewFilletEdge->m_cpCorner
                                                        //      NULL to ignore, default:[NULL]
      // set FilletEdgeType based on analysis done and saved in filletVertexType
      if (m_vVertices[1]->GetFilletVertexType() == SM_FV_RAIL_X_EXTENDED_EDGEUSE)
        {
          pNewFilletEdge2->SetFilletEdgeType( SM_FE_ON_EXTEND_EDGE );
          pNewFilletEdge2->SetOriginalEdge( pExtendEdge );
        }
      else
        {
          pNewFilletEdge2->SetFilletEdgeType( SM_FE_ON_EDGE );

          if (lTotalEUs <= 3)
            {
              // Register this edge and process it later
              // when removing topological vertices
              m_pExecutive->AddTopoEdges( pNewFilletEdge2 );
            }
          pNewFilletEdge2->SetOriginalEdge( pEU[1]->GetEdge() );
        }

      pNewFilletEdge2->SetOriginalFace( pEU[1]->GetFace() );
      m_vEdges.Add( pNewFilletEdge2 );

      // Add 2nd fillet edge from origVertex to other expected rail1/sideEdge intersection
      SmFilletEdge *pNewFilletEdge1 = NULL ;
      SER(SmFilletEdge::MakeFilletEdge(m_pPseudoBrep,   // in : target Brep to receive new topology objects
                                       m_vVertices[2],  // in : start of new FilletEdge
                                       m_vVertices[0],  // in : end   of new FilletEdge
                                       pNewFilletEdge1, // out: newly allocated FilletEdge - no Curve or UVTrimCurve data
                                       this));          // in : NewFilletEdge's filletCorner, gets stored in rpNewFilletEdge->m_cpCorner
                                                        //      NULL to ignore, default:[NULL]
      // set FilletEdgeType based on analysis done and saved in filletVertexType
      if (m_vVertices[0]->GetFilletVertexType() == SM_FV_RAIL_X_EXTENDED_EDGEUSE)
        {
          pNewFilletEdge1->SetFilletEdgeType( SM_FE_ON_EXTEND_EDGE );
          pNewFilletEdge1->SetOriginalEdge( pExtendEdge );
        }
      else
        {
          pNewFilletEdge1->SetFilletEdgeType( SM_FE_ON_EDGE );
          if ( lTotalEUs <= 3 )
            {
              m_pExecutive->AddTopoEdges( pNewFilletEdge1 );
            }
          pNewFilletEdge1->SetOriginalEdge( pEU[0]->GetEdge() );
        }
      pNewFilletEdge1->SetOriginalFace( pEU[0]->GetFace() );
      m_vEdges.Add( pNewFilletEdge1 );

     // all the corner topology is in place

     // When endFace connects to more than the filletSurface and 1 other Face,
     // check for gaps
     if ( bMultiNeighborEndFace )
       {
          SM_ASSERT(pNewFilletEdge1->GetFilletEdgeType() == SM_FE_ON_EDGE) ;
          SM_ASSERT(pNewFilletEdge2->GetFilletEdgeType() == SM_FE_ON_EDGE) ;

          // in this case we can set FilletVertex and SideFilletEdge shapes
          // without having the filletSurface - do so

          // set filletVertex->Positions
          CalcCornerVertGeom() ;

          // set FilletSideEdge->Curves
          pNewFilletEdge1->CalcCornerEdgeGeom() ;
          pNewFilletEdge2->CalcCornerEdgeGeom() ;

          // When proposing an extended surface
          if ( pNewExtendSurface )
            {
              // get extendedSurface/OtherSideEdge coincidence to within tolerance
              SmBoolean  bFoundCoincidence = FALSE ;
              SmSolution sSolution ;
              pNewExtendSurface->SimpleCoincidenceChecker(
                  pNewExtendSurface->GetNaturalUVDomain(),
                  *pOtherSideEdgeuse->GetEdge()->GetCurve(),
                  pOtherSideEdgeuse->GetEdge()->GetInterval(),
                  m_dThisApproxTol3d,
                  bFoundCoincidence,
                  sSolution
              );

              // when extendedSurface/OtherSideEdge are not coincident
              if( bFoundCoincidence == FALSE )
                {
                  // can't define endFace->Surface as a sideFace->ExtendedSurface
                  m_bExtendedSurfacePatch = FALSE ;

                }
            } // end pNewExtendSurface existence check
        } // end endFace connects to more than the filletSurface and 1 other Face check

    } // end bMixedCvxty check

  // all done
  return SM_SUCCESS;

} // end SmFilletNx1Corner::MakeCornerTopology

/*******************************************************************//**
PURPOSE: An Nx2 corner represents a corner with N adjacent
    edges and two of them are filleted. If N = 3, and all three edges
    have the same convexity, a bevel corner will be created.  For
    tangential corners, the two fillet surfaces should meet smoothly.
    A blending corner patch will be generated for mixed-convexity
    fillet corners.

//cbi: changing this:
    If N = 4, the two filleted edges should not share the same face
    and the corner will consists of only one intersection curve between
    two fillets just as in 3x2 case.


NOTES: Corners with N > 4 are not yet implemented.

***********************************************************************/
SmFilletNx2Corner::SmFilletNx2Corner
  (const SmContext      & crContext,
   const SmVertex       * cpVertex,
   SmTArray<SmEdgeuse*> * pSolverEUs,
   SmTArray<SmEdgeuse*> * pAllEUs,
   SmFilletExecutive    * pExec,
   double                 dApproxTol3d,
   double                 dTangencyTolRadians)
 : SmFilletCorner(crContext,
                  cpVertex,
                  pSolverEUs,
                  pAllEUs,
                  pExec,
                  dApproxTol3d,
                  dTangencyTolRadians)
{

} // end SmFilletNx2Corner::SmFilletNx2Corner constructor

/*******************************************************************//**
PURPOSE: Make corner topology which consists of an array of
    SmFilletVertex and an array of SmFilletEdge.

NOTES:
***********************************************************************/
SmStatus SmFilletNx2Corner::MakeCornerTopology()
{
int iDebugLevel = 0;
#ifdef SM_DEBUG_CODE
iDebugLevel = DebugLevel();
#endif // SM_DEBUG_CODE

  // locals
  ULONG ii;
  ULONG lTotalEUs = m_pAllEUs->GetSize();

  // no work - unhandled case - Currently we handle only 3x2 and 4x2
  if ( lTotalEUs > 4)
    { SER(SM_ERR) ; } 
     
  // Nx2 corner, get the two filleted edgeuses from the base brep
  SmEdgeuse * pBaseEdgeuse1 = (*m_pSolverEUs)[0];
  SmEdgeuse * pBaseEdgeuse2 = (*m_pSolverEUs)[1];

  // for 4 EU case
  if (lTotalEUs == 4)
    {
#ifdef SM_DEBUG_CODE
      if ( iDebugLevel > 0 ) 
        {
          SmBrep *pBrep = GetFilletExecutive()->GetTargetBrep() ;
          smgfx_SetLook(1,2, 0,0,1); if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
          smgfx_SetLook(3,4, 1,0,0); pBaseEdgeuse1->Draw(); sm_GraphicsLoop();
          smgfx_SetLook(3,4, 1,0,1); pBaseEdgeuse2->Draw(); sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE

      // check state: to be filleted edgeuses are on list of outward edgeuses surrounding the corner
      ULONG lFoundIndex1 = 0, lFoundIndex2 = 0;
      if (   !m_pAllEUs->FindElement( pBaseEdgeuse1, lFoundIndex1 )
          || !m_pAllEUs->FindElement( pBaseEdgeuse2, lFoundIndex2 ) )
        { SER(SM_ERR) ; }

      // Do a little test to make sure two filleted edgeuses
      // are not adjacent to each other
      if ( (lFoundIndex1+lFoundIndex2)%2 != 0 )
        {
          // 2 filleted edgeuses appeared next to each other
          // in the all-edgeuses array
          SER(SM_ERR);
        }

      // Check the non-filleted (side) edges for mixed convexity
      for (ii=0; ii<lTotalEUs; ii++)
        {
          SmEdgeuse * pSideEU = (*m_pAllEUs)[ii];

          // skip edgeuses to be filleted - looking for side (not filleted edgeuses)
          if ( pSideEU == pBaseEdgeuse1 || pSideEU == pBaseEdgeuse2 )
            { continue ; } // (not a side edge)

          // Classify a face's corner at a given vertex as convex or concave
          //            FALSE = inside face angle runs from 0 to 180+tol   degrees
          //            TRUE  = inside face angle runs from 180+tol to 360 degrees
          //  
          //         |\\ - face    \\\\\|  CONCAVE        Special Case: tangential edgeuses
          //  CONVEX |\\\   inside -\\\\|   FACE CORNER     Classified as CONVEX CORNER
          //         +-----          \\\+-----               -----+-----
          /*     FACE CORNER          \\\\\\\\                \\\\\\\\\\                   */
          /*                            \\\\\\\                \\\\\\\\\                   */
          /*                                                                               */  
          SmBoolean bMixedCvxty = sm_CheckFaceComplimentaryCornerConcavity
                                    (m_cpVertex,    // in : target corner vertex
                                     pBaseEdgeuse1, // in : 1st target edgeuse - points outward from CornerVert
                                     pBaseEdgeuse2, // in : 2nd target edgeuse - points outward from CornerVert
                                     pSideEU,       // in : NULL    = EU1 and EU2 are immediate neighbors bounding same face
                                                    //      notNULL = OptRefEU lies 'between' EU1 and EU2 so that EU1->edge and
                                                    //        EU2->edge do not bound the same face. Needs to be a tangential edge.
                                     iDebugLevel ); // in :

          if ( bMixedCvxty )
            {
              return Make4x2ConcaveCornerTopology();
            }
        }
      m_bBevel = TRUE ;

    } // end 4 outward EUs from this corner vertex case check

  // Determine if two edges are tangent at corner vertex.
  // Set bIsTangentCase, dAngleDeg, and bMixedConvexity.

  double dAngleDeg;
  double dTangencyTolDeg = SM_RAD2DEG( m_dTangencyTolRadians );
  SmBoolean bIsTangentCase = sm_TestG1Continuity          // rtn: TRUE = Edge EndTangents at vertex are within tolerance of being tangent
                               (m_cpVertex,               // in : Target Vertex
                                pBaseEdgeuse1->GetEdge(), // in : 1st edge connecting to vertex
                                pBaseEdgeuse2->GetEdge(), // in : 2nd edge connectint to vertex
                                dTangencyTolDeg,          // in : Tangency tolerance in degrees
                                &dAngleDeg,               // out: angle (degrees) between edges at vertex
                                iDebugLevel) ;            // in : 

  // Store this value for later use.
  m_bG1Case = bIsTangentCase;

  SmBoolean bMixedConvexity = sm_CheckFaceComplimentaryCornerConcavity
                                (m_cpVertex,     // in : target corner vertex
                                 pBaseEdgeuse1,  // in : 1st target edgeuse - points outward from CornerVert
                                 pBaseEdgeuse2,  // in : 2nd target edgeuse - points outward from CornerVert
                                 NULL,           // in : NULL    = EU1 and EU2 are immediate neighbors bounding same face
                                                 //      notNULL = OptRefEU lies 'between' EU1 and EU2 so that EU1->edge and
                                                 //        EU2->edge do not bound the same face. Needs to be a tangential edge.
                                 iDebugLevel) ;  // in :

  // Test if we need to create a corner surface for concave cases
  // Note: Only when angle is more than 5 degrees, cause otherwise
  // it will be a very skinny surface & may cause problems in later process
//    if (!bIsTangentCase && dAngleDeg < 5.0)
//    if (!bIsTangentCase && dAngleDeg > 5.0)
//

  // Get Solvers and FilletGeoms for the two filleted edges at this vertex
  SmFilletSolver * pFilSolver1  = GetFilletExecutive()->GetFilletSolverOfEdgeuse(pBaseEdgeuse1);
  SmFilletSolver * pFilSolver2  = GetFilletExecutive()->GetFilletSolverOfEdgeuse(pBaseEdgeuse2);
  SmFilletGeom   * pFilletGeom1 = pFilSolver1->GetFirstFilletGeom();
  SmFilletGeom   * pFilletGeom2 = pFilSolver2->GetFirstFilletGeom();

  // For the non-filleted edge(s), we expect the rails to hit it (them).
  // Create a (fillet) vertex there, along with two rail edgeuses.
  for (ii=0; ii<lTotalEUs; ii++)
    {
      SmEdgeuse * pCurrEU = (*m_pAllEUs)[ii];

      if(   pCurrEU != pBaseEdgeuse1 
         && pCurrEU != pBaseEdgeuse2 )
        {
          // This edgeuse is in between two filletted edgeuses
          SmEdgeuse      * pSideEU    = sm_FindSideEdgeuse( pBaseEdgeuse1, pCurrEU, iDebugLevel );

          //
          SmFilletVertex * pNewSideFV = new (m_pPseudoBrep) SmFilletVertex( this );
          pNewSideFV->SetPointClass( SM_PC_EDGEUSE, pSideEU );
          pNewSideFV->SetFilletVertexType( SM_FV_RAIL_X_EDGEUSE );
          m_vVertices.Add( pNewSideFV );

          ULONG             lRailIndex      = pFilSolver1->FindIndexOfRailXSideEdgeuse( pSideEU );
          ULONG             lOtherRailIndex = pFilSolver2->FindIndexOfRailXSideEdgeuse(pSideEU->GetRadial() );
          SmFilletEdgeuse * pNewEdgeuse     = NULL;

          // Make a FilletEdgeuse and a FilletVertexuse to connect 
          // a given filletVertex to one of this FilletGeom's Fillet Rails.
          SER( pFilletGeom1->MakeRailEdgeuse( m_pPseudoBrep, lRailIndex,      pNewSideFV, pNewEdgeuse ));
          SER( pFilletGeom2->MakeRailEdgeuse( m_pPseudoBrep, lOtherRailIndex, pNewSideFV, pNewEdgeuse ));

        } // end pCurrEU is a side EU check
    } // end iter all outgoing edgeuses from this vertex 

  // Create the topology for the fillet corner.
  if ( lTotalEUs == 3 )
    {
      // In the normal uniform-convexity case, two rails will intersect
      // in the common face, and the other two rails should intersect the
      // un-filleted side edge at the same place.
      // We make vertices at those two places, and an edge between them.
      // For tangent and mixed-convexity cases ... see comments below.

      // gwc: assuming two rails should intersect the un-filleted side edge AT THE SAME PLACE isn't always true.  
      // This gets detected and processed with virtual call SmFilletCorner::AdjustTopology() made in SmFilletExecutive::CreateFilletBrep().

      // Find the side EU -- the un-filleted one.
      SmEdgeuse * pSideEU = NULL;
      for(ii=0;ii<lTotalEUs;ii++)
        {
          SmEdgeuse * pCurrEU = (*m_pAllEUs)[ii];
          if ( pCurrEU != pBaseEdgeuse1 && pCurrEU != pBaseEdgeuse2 )
            {
              pSideEU = pCurrEU;
              break;
            }
        } // end iter all EUs looking for the side (nonFilleted) edgeuse

      // locals
      SmFace          * pCommonFace     = sm_GetFaceByTwoEdgeuses(pBaseEdgeuse1, pBaseEdgeuse2, pSideEU );
      ULONG             lRailIndex      = pFilSolver1->FindIndexOfRailOnFace(pCommonFace);
      ULONG             lOtherRailIndex = pFilSolver2->FindIndexOfRailOnFace(pCommonFace);
      SmFilletVertex  * pNewSideFV      = m_vVertices[0];
      SmFilletVertex  * pNewFaceFV      = new (m_pPseudoBrep) SmFilletVertex(this);
      SmFilletEdgeuse * pNewEdgeuse     = NULL;

      pNewFaceFV->SetPointClass( SM_PC_FACE, pCommonFace );
      m_vVertices.Add( pNewFaceFV );

      // mixed convexity case
      if(   bMixedConvexity 
         && !m_bBevel 
         && dAngleDeg > 5.0 )
        {
          // Mixed-convexity 3x2 corner, will create a 3-sided corner face.
          m_bBlending = TRUE;

          // It will just be straight across from where the other rail
          // hits the side edge (at pNewSideFV):
          pNewFaceFV->SetFilletVertexType( SM_FV_MATE );
          pNewFaceFV->SetMate( 0, pNewSideFV );
          pNewSideFV->SetMate( 0, pNewFaceFV );

          // Make a FilletEdgeuse and a FilletVertexuse to connect 
          // a given filletVertex to one of this FilletGeom's Fillet Rails.
          SER( pFilletGeom1->MakeRailEdgeuse( m_pPseudoBrep, lRailIndex, pNewFaceFV, pNewEdgeuse ));

          // pNewFaceFV, in this case, will not be where the two rails meet
          // in the common face, because they're not going to meet, we're
          // going to put in a 3-sided face in between them.
          // pNewFaceFV will be where one rail meets the 3-sided face,
          // and we'll need another vertex for the other rail meeting it,
          // plus two new edges.

          SmFilletVertex * pNewFaceFV2 = new (m_pPseudoBrep) SmFilletVertex(this);
          m_vVertices.Add( pNewFaceFV2 );
          pNewFaceFV2->SetPointClass( SM_PC_FACE, pCommonFace ) ;

          // It will just be straight across from where the other rail
          // hits the side edge (at pNewSideFV):
          pNewFaceFV2->SetFilletVertexType( SM_FV_MATE );
          pNewFaceFV2->SetMate( 0, pNewSideFV );
          pNewSideFV-> SetMate( 1, pNewFaceFV2 );

          // Make a FilletEdgeuse and a FilletVertexuse to connect 
          // a given filletVertex to one of this FilletGeom's Fillet Rails.
          SER( pFilletGeom2->MakeRailEdgeuse( m_pPseudoBrep, lOtherRailIndex, pNewFaceFV2, pNewEdgeuse ));

          // Make Fillet Edge: a straight-across rib to end filletGeom1.
          SmFilletEdge * pNewRibFilEdge = NULL;
          SER( SmFilletEdge::MakeFilletEdge( m_pPseudoBrep,  // in : target Brep to receive new topology objects
                                             pNewSideFV,     // in : start of new FilletEdge
                                             pNewFaceFV,     // in : end   of new FilletEdge
                                             pNewRibFilEdge, // out: newly allocated FilletEdge - no Curve or UVTrimCurve data
                                             this ));        // in : NewFilletEdge's filletCorner, gets stored in rpNewFilletEdge->m_cpCorner
                                                             //      NULL to ignore, default:[NULL]

          pNewRibFilEdge->SetFilletEdgeType( SM_FE_CROSS_SECTION );
          SmFilletEdgeuse * pPrimEU      = (SmFilletEdgeuse*)pNewRibFilEdge->GetPrimaryEdgeuse();
          SmFilletGeom    * pFilletGeom3 = pFilSolver1->GetFirstFilletGeom();
          pPrimEU->SetFilletGeom( pFilletGeom3 );
          m_vEdges.Add( pNewRibFilEdge );

          // Make the 3-sided face's third edge.
          // This connects the two rails, and will lie inside the common face.
          // (Well, eventually it will separate the common face from the new 3-sided face.)
          SER( SmFilletEdge::MakeFilletEdge( m_pPseudoBrep,   // in : target Brep to receive new topology objects
                                             pNewFaceFV,      // in : start of new FilletEdge
                                             pNewFaceFV2,     // in : end   of new FilletEdge
                                             pNewRibFilEdge,  // out: newly allocated FilletEdge - no Curve or UVTrimCurve data
                                             this ));         // in : NewFilletEdge's filletCorner, gets stored in rpNewFilletEdge->m_cpCorner
                                                              //      NULL to ignore, default:[NULL]

          pNewRibFilEdge->SetFilletEdgeType( SM_FE_RAIL_RAIL_INTERPOLATION );
          pNewRibFilEdge->SetOriginalFace( pCommonFace );
          m_vEdges.Add( pNewRibFilEdge );

          // Make a straight-across rib to end filletGeom2.
          SER( SmFilletEdge::MakeFilletEdge( m_pPseudoBrep,   // in : target Brep to receive new topology objects
                                             pNewFaceFV2,     // in : start of new FilletEdge
                                             pNewSideFV,      // in : end   of new FilletEdge
                                             pNewRibFilEdge,  // out: newly allocated FilletEdge - no Curve or UVTrimCurve data
                                             this ));         // in : NewFilletEdge's filletCorner, gets stored in rpNewFilletEdge->m_cpCorner
                                                              //      NULL to ignore, default:[NULL]

          pNewRibFilEdge->SetFilletEdgeType( SM_FE_CROSS_SECTION );
          pPrimEU                     = (SmFilletEdgeuse*)pNewRibFilEdge->GetPrimaryEdgeuse();
          SmFilletGeom * pFilletGeom4 = pFilSolver2->GetFirstFilletGeom();
          pPrimEU->SetFilletGeom( pFilletGeom4 );
          m_vEdges.Add( pNewRibFilEdge );

          // all done
          return SM_SUCCESS;

        }  // end mixed-concavity case branch.

      //
      if (   bIsTangentCase
          // || dAngleDeg <= 2.0
          || (bMixedConvexity && dAngleDeg <= 5.0) )
        {
          m_bBlending = TRUE;
          // Two filleted edgeuses are tangent to each other.
          // The new face-vertex will just be straight across from the
          // rail-side edge intersection vertex.
          pNewFaceFV->SetFilletVertexType( SM_FV_MATE );
          pNewFaceFV->SetMate( 0, pNewSideFV );
          pNewSideFV->SetMate( 0, pNewFaceFV );
        }
      else // Regular BEVEL corner branch
        {
          // The new face vertex will be the intersection of the rails.
          pNewFaceFV->SetFilletVertexType( SM_FV_RAIL_X_RAIL );
          m_bBevel = TRUE;
        } // end regular BEVEL corner branch

      // Setup rails of fillet
      //   Make a FilletEdgeuse and a FilletVertexuse to connect 
      //   a given filletVertex to one of this FilletGeom's Fillet Rails.
      SER( pFilletGeom1->MakeRailEdgeuse( m_pPseudoBrep, lRailIndex, pNewFaceFV, pNewEdgeuse));
      SER( pFilletGeom2->MakeRailEdgeuse( m_pPseudoBrep, lOtherRailIndex, pNewFaceFV, pNewEdgeuse));

    }  // end lTotalEUs == 3 branch

  else if ( lTotalEUs == 4 )
    {
      // Note, at this point we don't know what the topology will be.
      // There are side-edges between both pairs of rails.
      // (Note, we're assuming exactly one side-edge between each
      // potentially-intersecting rails: we exluded other case above.)
//cbi: haven't implemented this yet:
      // If the two side-edge-rail-rail intersections are straight across
      // from each other, then the two fillet surfaces will meet, at a
      // single rib.  Otherwise, we will create a third fillet face to join
      // the two.

      // Set m_bBlending and m_bBevel in 4x2 case as well. [bd 050718]
      if (   bIsTangentCase
                           //dAngleDeg <= 2.0
          || (bMixedConvexity && dAngleDeg <= 5.0) )
        {
          // Two filleted edgeuses are tangent to each other
          m_bBlending = TRUE;
          m_bBevel = FALSE;
        }
      else
        {
          // Regular BEVEL corner
          m_bBevel = TRUE;
          m_bBlending = FALSE;
        }
    }  // end lTotalEUs == 4 branch


  // At this point, we have only one fillet (cross) edge at this corner.
  // In the 3x2 case, it will run from the rail-rail intersection of the
  // two rails in the common face to the rail/rail/side-edge intersection.
  if(m_vVertices.GetSize() != 2)
    { SER(SM_ERR) ; }

  // Create the new cross-edge.
  SmFilletEdge * pNewFilletCrossEdge = NULL;
  SER( SmFilletEdge::MakeFilletEdge(m_pPseudoBrep,        // in : target Brep to receive new topology objects
                                    m_vVertices[0],       // in : start of new FilletEdge
                                    m_vVertices[1],       // in : end   of new FilletEdge
                                    pNewFilletCrossEdge,  // out: newly allocated FilletEdge - no Curve or UVTrimCurve data
                                    this ));              // in : NewFilletEdge's filletCorner, gets stored in rpNewFilletEdge->m_cpCorner
                                                          //      NULL to ignore, default:[NULL]
  SmFilletEdgeuse * pPrimEU = (SmFilletEdgeuse*)pNewFilletCrossEdge->GetPrimaryEdgeuse() ;
  SmFilletEdgeuse * pMateEU = (SmFilletEdgeuse*)pPrimEU->GetMate() ;
  pPrimEU->SetFilletGeom(pFilletGeom1) ;
  pMateEU->SetFilletGeom(pFilletGeom2) ;

  // Note: in version 5, this said 'if ( bIsTangentCase )', but was changed
  // to 'if ( m_bBlending )'.  In the 3x2 case, they're the same, but not
  // in the 4x2 case.  'Or' is correct here.
  // [bd 09 Nov 05, 050718]

  // if (m_bBlending)
  if ( bIsTangentCase || m_bBlending )
    {
      pNewFilletCrossEdge->SetFilletEdgeType( SM_FE_CROSS_SECTION );
    }
  else
    {
      pNewFilletCrossEdge->SetFilletEdgeType( SM_FE_FILLET_X_FILLET );
    }

  m_vEdges.Add( pNewFilletCrossEdge );

  // all done
  return SM_SUCCESS;

} // end SmFilletNx2Corner::MakeCornerTopology

/*******************************************************************//**
PURPOSE: If this FilletCorner has exactly two FilletSolvers, and their Edges
   meet with G1 continuity, return the other one than the passed-in argument.

NOTES:
   If the passed-in FilletSolver is not one of ours, return Null with an error.
   If there is only one FilletSolver, and it is the one passed in, return it.
***********************************************************************/
SmFilletSolver * SmFilletNx2Corner::GetTangentFilletSolver( const SmFilletSolver *pFS )
{
  if ( m_pSolverEUs->GetSize() != 2 )  { SM_ASSERT_ERR; return NULL; }

  // We have to test its continuity.
  SmEdgeuse      * pEU0 = (*m_pSolverEUs)[0];
  SmEdge *pEdge0 = pEU0->GetEdge();
  SmEdgeuse      * pEU1 = (*m_pSolverEUs)[1];
  SmEdge *pEdge1 = pEU1->GetEdge();

  double dTangencyTolDeg = SM_RAD2DEG( m_dTangencyTolRadians );
  int iDebugLevel = 0;  // ... enhance as needed.
  if ( ! sm_TestG1Continuity( m_cpVertex, pEdge0, pEdge1, dTangencyTolDeg, NULL, iDebugLevel ))
    { return NULL; }   // not an error.


  SmFilletSolver * pFS0 = m_pExecutive->GetFilletSolverOfEdgeuse( pEU0 );
  SmFilletSolver * pFS1 = m_pExecutive->GetFilletSolverOfEdgeuse( pEU1 );

  if ( pFS0 == pFS )
    { return pFS1; }
  if ( pFS1 == pFS )
    { return pFS0; }

  SM_ASSERT_ERR; return NULL;

} // end SmFilletNx2Corner::GetTangentFilletSolver

/*******************************************************************//**
PURPOSE: Make corner topology for 4x2-Concave Corner cases.
    These (rare-)cases are very similiar to 3x2-Concave cases except that
    an extra edge (very likely a topological edge) is connected to
    the corner vertex

NOTES:
***********************************************************************/
SmStatus SmFilletNx2Corner::Make4x2ConcaveCornerTopology
  ()
{
int iDebugLevel = 0;
#ifdef SM_DEBUG_CODE
iDebugLevel = DebugLevel();
#endif // SM_DEBUG_CODE

    SmEdgeuse * pBaseEdgeuse1 = (*m_pSolverEUs)[0];
    SmEdgeuse * pBaseEdgeuse2 = (*m_pSolverEUs)[1];

    // Get Solvers
    SmFilletSolver * pFilSolver1 = GetFilletExecutive()->
        GetFilletSolverOfEdgeuse(pBaseEdgeuse1);
    SmFilletSolver * pFilSolver2 = GetFilletExecutive()->
        GetFilletSolverOfEdgeuse(pBaseEdgeuse2);
    SmFilletGeom * pFilletGeom1 = pFilSolver1->GetFirstFilletGeom();
    SmFilletGeom * pFilletGeom2 = pFilSolver2->GetFirstFilletGeom();

    SmFace * pCommonFace = NULL;
    SmFilletVertex * pFVOnCommonFace = NULL;
    SmFilletVertex * pFVOnConvexEdge = NULL;
    SmFilletGeom * pConvexFG = NULL;
    SmFilletGeom * pConcaveFG = NULL;

    // Make topology for all (both) of the non-filleted edges at this vertex.

    ULONG ii;
    for (ii=0; ii<4; ii++)
    {
        SmEdgeuse * pCurrEU = (*m_pAllEUs)[ii];
        if (pCurrEU == pBaseEdgeuse1 || pCurrEU == pBaseEdgeuse2)
        {
            continue;  // Do only the non-filleted ones.
        }

        // This edgeuse is in between two filletted edgeuses
        SmFilletVertex * pFV = new (m_pPseudoBrep) SmFilletVertex(this);
        m_vVertices.Add(pFV);

        SmEdgeuse * pSideEU = sm_FindSideEdgeuse(pBaseEdgeuse1, pCurrEU, iDebugLevel );
        pFV->SetPointClass(SM_PC_EDGEUSE,pSideEU);
        pFV->SetFilletVertexType(SM_FV_RAIL_X_EDGEUSE);

        SmFilletVertex * pFV2 = NULL;

        if ( sm_CheckFaceComplimentaryCornerConcavity(
                m_cpVertex, pBaseEdgeuse1, pBaseEdgeuse2, pCurrEU, iDebugLevel )
           )
        {
            double dAngle1 = sm_FindAngleBetweenEdgeuses(
                    m_cpVertex, pCurrEU, pBaseEdgeuse1, iDebugLevel );

            double dAngle2 = sm_FindAngleBetweenEdgeuses(
                    m_cpVertex, pCurrEU, pBaseEdgeuse2, iDebugLevel );

            //if (sm_CheckFaceComplimentaryCornerConcavity(
                    //m_cpVertex, pCurrEU, pBaseEdgeuse1, NULL, iDebugLevel ))

            if ( dAngle1 > dAngle2 )
            {
                pCommonFace = sm_GetFaceByTwoEdgeuses(
                    pCurrEU,pBaseEdgeuse1,NULL);
                pFV->SetPointClass(SM_PC_EDGEUSE,pSideEU->GetRadial());

                pFVOnCommonFace = new (m_pPseudoBrep) SmFilletVertex(this);
                pFVOnCommonFace->SetPointClass(SM_PC_FACE, pCommonFace);

                pFV2 = pFVOnCommonFace;
                pFV2->SetFilletVertexType(SM_FV_MATE);
                SM_SWAP_PTR(SmFilletVertex,pFV,pFV2);
                pConcaveFG = pFilletGeom1;
                pConvexFG = pFilletGeom2;
            }

            //else if (sm_CheckFaceComplimentaryCornerConcavity(
                    //m_cpVertex, pCurrEU, pBaseEdgeuse2, NULL, iDebugLevel ))

            else // if ( dAngle1 < dAngle2 )  [bd 060424]
            {
                pCommonFace = sm_GetFaceByTwoEdgeuses(
                    pCurrEU,pBaseEdgeuse2,NULL);

                pFVOnCommonFace = new (m_pPseudoBrep) SmFilletVertex(this);
                pFVOnCommonFace->SetPointClass(SM_PC_FACE, pCommonFace);

                pFV2 = pFVOnCommonFace;
                pFV2->SetFilletVertexType(SM_FV_MATE);
                pConcaveFG = pFilletGeom2;
                pConvexFG = pFilletGeom1;
            }
        }
        else
        {
            pFV2 = pFV;
            pFVOnConvexEdge = pFV;
        }

        ULONG lRailIndex = pFilSolver1->FindIndexOfRailXSideEdgeuse(pSideEU);
        SmFilletEdgeuse * pNewEdgeuse = NULL;
        SER(pFilletGeom1->MakeRailEdgeuse(m_pPseudoBrep,
            lRailIndex,pFV,pNewEdgeuse));
        ULONG lOtherRailIndex = pFilSolver2->FindIndexOfRailXSideEdgeuse(
            pSideEU->GetRadial());
        SER(pFilletGeom2->MakeRailEdgeuse(m_pPseudoBrep,
            lOtherRailIndex,pFV2,pNewEdgeuse));

    } // end loop on all (both) non-filleted edges.

    NER(pFVOnCommonFace);
    NER(pFVOnConvexEdge);
    pFVOnCommonFace->SetMate(0, pFVOnConvexEdge);
    pFVOnConvexEdge->SetMate(0, pFVOnCommonFace);
    m_vVertices.Add(pFVOnCommonFace);

    // Will create a 3-sided corner face
    SmFilletVertex * pPrevFV = pFVOnCommonFace; //i.e. m_vVertices[2]
    for (ii=0; ii<3; ii++)
    {
        SmFilletVertex * pCurrFV = m_vVertices[ii];
        SmFilletEdge * pFilletEdge = NULL;
        if (ii==1 || pCurrFV == pFVOnConvexEdge || pPrevFV == pFVOnConvexEdge)
        {
            // Make Fillet Edge
            SER(SmFilletEdge::MakeFilletEdge(m_pPseudoBrep,  // in : target Brep to receive new topology objects
                                             pPrevFV,        // in : start of new FilletEdge
                                             pCurrFV,        // in : end   of new FilletEdge
                                             pFilletEdge,    // out: newly allocated FilletEdge - no Curve or UVTrimCurve data
                                             this));         // in : NewFilletEdge's filletCorner, gets stored in rpNewFilletEdge->m_cpCorner
                                                             //      NULL to ignore, default:[NULL]
            pFilletEdge->SetFilletEdgeType(SM_FE_CROSS_SECTION);
            SmFilletEdgeuse * pPrimEU = (SmFilletEdgeuse*)pFilletEdge->
                GetPrimaryEdgeuse();
            if (ii == 1) {
                pPrimEU->SetFilletGeom(pConvexFG);
            }
            else {
                pPrimEU->SetFilletGeom(pConcaveFG);
            }
        }
        else
        {
            SER(SmFilletEdge::MakeFilletEdge(m_pPseudoBrep,  // in : target Brep to receive new topology objects
                                             pPrevFV,        // in : start of new FilletEdge
                                             pCurrFV,        // in : end   of new FilletEdge
                                             pFilletEdge,    // out: newly allocated FilletEdge - no Curve or UVTrimCurve data
                                             this));         // in : NewFilletEdge's filletCorner, gets stored in rpNewFilletEdge->m_cpCorner
                                                             //      NULL to ignore, default:[NULL]
            pFilletEdge->SetFilletEdgeType(SM_FE_RAIL_RAIL_INTERPOLATION);
            pFilletEdge->SetOriginalFace(pCommonFace);
        }
        m_vEdges.Add(pFilletEdge);
        pPrevFV = pCurrFV;
    }

    return SM_SUCCESS;

} // end SmFilletNx2Corner::Make4x2ConcaveCornerTopology

/*******************************************************************//**
PURPOSE: Adjust the corner topology for the mixed-convexity cases where two fillets
   of the corner does not intersect each other on the side edge.
   One additional corner edge will be added to fill the 'gap'.

NOTES:
***********************************************************************/
SmStatus SmFilletNx2Corner::AdjustConcaveCornerTopology()
{
  // Find the fillet vertex to be adjusted, pAdjustVert.
  // It's the only SM_FV_RAIL_X_EDGEUSE.

  SmFilletVertex * pAdjustVert = NULL;
  SmTArray<SmFilletGeom*> sGeoms;
  ULONG ii;
  for ( ii=0; ii<m_vVertices.GetSize(); ii++ )
  {
      SmFilletVertex * pV = m_vVertices[ii];
      if ( pV->GetFilletVertexType() != SM_FV_RAIL_X_EDGEUSE )
        { continue; }

      pV->GetFilletGeoms( sGeoms );
      if ( sGeoms.GetSize() != 2 )
      { continue; }  //SER(SM_ERR);
      pAdjustVert = pV;
      break;
  }
  NER(pAdjustVert);

  SmFilletGeom * pFilletGeom1 = sGeoms[0];
  SmFilletGeom * pFilletGeom2 = sGeoms[1];
  SmEdgeuse * pSideEU1 = (SmEdgeuse*)pAdjustVert->GetPointClassObject();
  SmFilletVertexuse * pVU1 = pAdjustVert->GetVUAtRailEnd( pFilletGeom1 );
  SmFilletVertexuse * pVU2 = pAdjustVert->GetVUAtRailEnd( pFilletGeom2, pVU1 );
  double dEdgeParam1 = pVU1->GetTsectPnt().m_dCurveParameter;
  double dEdgeParam2 = pVU2->GetTsectPnt().m_dCurveParameter;
  SmPoint3d sPnt1, sPnt2;
  SmCurve * pSideEdgeCurve = pSideEU1->GetEdge()->GetCurve();
  pSideEdgeCurve->EvaluatePoint( dEdgeParam1,sPnt1 );
  pSideEdgeCurve->EvaluatePoint( dEdgeParam2,sPnt2 );

  // See if we have coincident geometry, i.e. two rails intersect at this vertex.
  double dGapDist = sPnt1.DistanceBetween( sPnt2 );
  if ( dGapDist < m_dThisApproxTol3d )
  {
      // No adjustments are needed
      return SM_SUCCESS;
  }

#ifdef SM_DEBUG_CODE
  if ( DebugLevel() > 0 ) {
      smgfx_SetLook(1,2, 1,0,0); pAdjustVert->Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop();
  }
#endif // SM_DEBUG_CODE

  // Create two new FilletVertices where each rail hits the side edge.
  SmFilletVertex * pNewVertL = new (m_pPseudoBrep) SmFilletVertex( this );  // "Large"
  pNewVertL->SetFilletVertexType( SM_FV_RAIL_X_EDGEUSE );
  SmFilletVertex * pNewVertS = new (m_pPseudoBrep) SmFilletVertex( this );  // "Small"
  pNewVertS->SetFilletVertexType( SM_FV_RAIL_X_EDGEUSE );
  m_vVertices.Add( pNewVertL );
  m_vVertices.Add( pNewVertS );

  pAdjustVert->SetFilletVertexType( SM_FV_FILLET_X_FILLET );
  pAdjustVert->SetStatus( SM_FIL_UNPROCESSED );

  // See which of the two rail-side edge intersections is closer
  // to the filleted vertex, to decide Large/Small.
  SmPoint3d sVertPnt = m_cpVertex->GetPoint();
  SmFilletGeom * pLargeFG = NULL;
  SmFilletGeom * pSmallFG = NULL;
  SmVertexuse * pVUOnLargeFG = NULL;
  SmVertexuse * pVUOnSmallFG = NULL;
  SmFace * pSideFace = NULL;
  if ( sPnt1.DistanceBetween( sVertPnt ) < sPnt2.DistanceBetween( sVertPnt ) )
  {
      pNewVertL->SetPoint( sPnt2 );
      pNewVertL->SetOriginalTParam( dEdgeParam2 );
      pNewVertS->SetPoint( sPnt1 );
      pNewVertS->SetOriginalTParam( dEdgeParam1 );
      pLargeFG = pFilletGeom2;
      pSmallFG = pFilletGeom1;
      pVUOnLargeFG = pVU2;
      pVUOnSmallFG = pVU1;
      pSideFace = pSideEU1->GetFace();
  }
  else
  {
      pNewVertL->SetPoint( sPnt1 );
      pNewVertL->SetOriginalTParam( dEdgeParam1 );
      pNewVertS->SetPoint( sPnt2 );
      pNewVertS->SetOriginalTParam( dEdgeParam2 );
      pLargeFG = pFilletGeom1;
      pSmallFG = pFilletGeom2;
      pVUOnLargeFG = pVU1;
      pVUOnSmallFG = pVU2;
      pSideFace = pSideEU1->GetRadial()->GetFace();
  }
  pNewVertL->SetStatus( SM_FIL_PROCESSED );
  pNewVertS->SetStatus( SM_FIL_PROCESSED );
  pAdjustVert->Remove( pVUOnLargeFG );
  SER( pNewVertL->PostInsert( pVUOnLargeFG ));
  pAdjustVert->Remove( pVUOnSmallFG );
  SER( pNewVertS->PostInsert( pVUOnSmallFG ));

  SmFilletEdge * pNewEdge = NULL;
  SER( SmFilletEdge::MakeFilletEdge(m_pPseudoBrep, // in : target Brep to receive new topology objects
                                    pAdjustVert,   // in : start of new FilletEdge
                                    pNewVertL,     // in : end   of new FilletEdge
                                    pNewEdge,      // out: newly allocated FilletEdge - no Curve or UVTrimCurve data
                                    this));        // in : NewFilletEdge's filletCorner, gets stored in rpNewFilletEdge->m_cpCorner
                                                   //      NULL to ignore, default:[NULL]
  pNewEdge->SetFilletEdgeType( SM_FE_FILLET_X_SIDE_FACE );
  pNewEdge->SetOriginalFace( pSideFace );
  SmFilletEdgeuse * pPrimEU = (SmFilletEdgeuse*)pNewEdge->GetPrimaryEdgeuse();
  pPrimEU->SetFilletGeom( pLargeFG );

  // Need to extend the side surface
  SmSurface * pSideSurface = pSideFace->GetSurface();
  SmSurface * pNewSurface = NULL;
  SER( pSideSurface->CreateExtendedSurface( m_crContext, 10.0*dGapDist, SM_CT_G1, pNewSurface) );
  SmObjDelete sCleanSurf( pNewSurface );
  //SmBSplineSurface * pSideSurface = SM_CAST_PTR( SmBSplineSurface, pSideFace->GetSurface() );
  //SmBSplineSurface * pNewSurface = NULL;
  //SER( pSideSurface->CreateExtendedSurface( m_crContext, 10.0*dGapDist, SM_CT_G1, pNewSurface) );
  //SmObjDelete sCleanSurf( pNewSurface );
  SmSurface *pNewAnalyticSurface;

  // Copy pNewSurface, when possible as an analytic surface.
  SER( pNewSurface->CopyAndAddAnalytics( m_crContext, pNewAnalyticSurface ));
  AddExtendedOriginalSurface(pSideFace,pNewAnalyticSurface);
  pNewAnalyticSurface = NULL; // to be safe: might have been deleted.

  SmFilletEdge * pNewEdge1 = NULL;
  SER( SmFilletEdge::MakeFilletEdge(m_pPseudoBrep,  // in : target Brep to receive new topology objects
                                    pNewVertL,      // in : start of new FilletEdge
                                    pNewVertS,      // in : end   of new FilletEdge
                                    pNewEdge1,      // out: newly allocated FilletEdge - no Curve or UVTrimCurve data
                                    this));         // in : NewFilletEdge's filletCorner, gets stored in rpNewFilletEdge->m_cpCorner
                                                    //      NULL to ignore, default:[NULL]
  pNewEdge1->SetFilletEdgeType( SM_FE_ON_EDGE );
  pNewEdge1->SetOriginalEdge( pSideEU1->GetEdge() );
  pNewEdge1->SetOriginalFace( pSideFace );
  // Set this edge as a topo edges so that it can be removed in the merge process
  //m_pExecutive->AddTopoEdges( pNewEdge1 );

  SmFilletEdge * pNewEdge2 = NULL;
  SER( SmFilletEdge::MakeFilletEdge(m_pPseudoBrep, // in : target Brep to receive new topology objects
                                    pNewVertS,     // in : start of new FilletEdge
                                    pAdjustVert,   // in : end   of new FilletEdge
                                    pNewEdge2,     // out: newly allocated FilletEdge - no Curve or UVTrimCurve data
                                    this));        // in : NewFilletEdge's filletCorner, gets stored in rpNewFilletEdge->m_cpCorner
                                                   //      NULL to ignore, default:[NULL]
  pNewEdge2->SetFilletEdgeType( SM_FE_RAIL_EXTENSION );
  pNewEdge2->SetOriginalFace( pSideFace );
  SmFilletEdgeuse * pPrimEU2 = (SmFilletEdgeuse*)pNewEdge2->GetPrimaryEdgeuse();
  pPrimEU2->SetFilletGeom( pSmallFG );

  m_vEdges.Add( pNewEdge  );
  m_vEdges.Add( pNewEdge1 );
  m_vEdges.Add( pNewEdge2 );

  // pAdjustVert comes in with two mates, in the common face.
  // Find them -- they are at the other end of the two FilletEdges
  // of type SM_FE_CROSS_SECTION -- and reset them as mates of
  // the two FilletVertices we just created.
  //SmVertexuse * pSmallFilletVU = NULL;
  //SmVertexuse * pLargeFilletVU = NULL;
  SmTArray<SmVertexuse*> sVUs;
  pAdjustVert->GetVertexuses( sVUs );
  for ( ii=0; ii<sVUs.GetSize(); ii++ )
  {
      SmVertexuse * pVU = sVUs[ii];
      SmFilletEdgeuse * pEU = (SmFilletEdgeuse*)pVU->GetEdgeuse();
      SmFilletEdge * pFilletE = (SmFilletEdge*)pEU->GetEdge();
      if ( pFilletE->GetFilletEdgeType() != SM_FE_CROSS_SECTION )
        { continue; }

      // FilletEdge of type SM_FE_CROSS_SECTION:
      SmFilletVertex * pOtherVert = (SmFilletVertex*)pFilletE->GetOtherVertex( pAdjustVert );
      SmFilletGeom * pFG = pEU->GetFilletGeom();
      if ( pFG == NULL )
      {
          pEU = (SmFilletEdgeuse*)pEU->GetMate();
          pFG = pEU->GetFilletGeom();
          NER( pFG );
      }
      if ( pFG == pSmallFG && pOtherVert->GetMate(0) != NULL )
      {
          pNewVertS->SetMate(0,pOtherVert);
          pOtherVert->SetMate(0,pNewVertS);
      }
      else if ( pFG == pLargeFG && pOtherVert->GetMate(0) != NULL )
      {
          pNewVertL->SetMate(0,pOtherVert);
          pOtherVert->SetMate(0,pNewVertL);
      }
  }
  pAdjustVert->SetMate( 0, NULL );
  pAdjustVert->SetMate( 1, NULL );

  // Allow fillets to be extended at the corner
  m_bBlending = FALSE;

  m_bTopologyAdjusted = TRUE;

  return SM_SUCCESS;

} // end SmFilletNx2Corner::AdjustConcaveCornerTopology


/*******************************************************************//**
PURPOSE: Adjust an Nx1 closed corner for when the two ends do not match up geometrically.

NOTES:
***********************************************************************/
SmStatus SmFilletNx1ClosedCorner::AdjustTangentCorner()
{
  // First check that this is the appropriate type.
  // Must be Nx1 tangent corner, and currently N must be 3.
  if ( m_pSolverEUs == NULL  ||  m_pSolverEUs->GetSize() != 1 ) { return SM_SUCCESS; }
  if ( m_pAllEUs    == NULL  ||  m_pAllEUs   ->GetSize() != 3 ) { return SM_SUCCESS; }

  // Check that the two filleted edges are tangent at this vertex.

  // Note: using NormalizedEvaluate() assumes that m_pSolverEUs go outward
  // from the filleted vertex.
  SmPoint3d sPt;
  SmVector3d s1stDeriv1, s1stDeriv2;
  (*m_pSolverEUs)[0]->NormalizedEvaluate( 0.0, FALSE, sPt, &s1stDeriv1 );   // TRUE = UV Eval, FALSE = 3d Eval
  (*m_pSolverEUs)[0]->NormalizedEvaluate( 1.0, FALSE, sPt, &s1stDeriv2 );   // TRUE = UV Eval, FALSE = 3d Eval

  double dAngTolDeg = 2.0; //cbi: some places use 5 degrees; sort that all out.
  if ( !s1stDeriv1.IsParallelTo( s1stDeriv2, dAngTolDeg)  )
      { return SM_SUCCESS; }


  // Ok, we are an Nx1-closed tangent corner.

  // Check whether the fillet surface meets itself smoothly: corner points will match.
  SmPoint3d sSrfPt00, sSrfPt01, sSrfPt10, sSrfPt11;
  // There will be two FilletGeoms, but they are the same.
  // It contains the Fillet surface, rails, etc.
  SmTArray< SmFilletGeom* > sFilletGeoms;
  m_vVertices[0]->GetFilletGeoms( sFilletGeoms );
  SM_ASSERT( sFilletGeoms.GetSize() == 2 ); // We know we're a 4x2 tangent corner.
  SM_ASSERT( sFilletGeoms[0] == sFilletGeoms[1] );
  SmFilletGeom *pOrigFG = sFilletGeoms[0];  // Call it 'Original' because we might split it.
  SmSurface *pFilletSrf = pOrigFG->GetFilletSurface();

  SmExtent2d sSrfDomain = pFilletSrf->GetNaturalUVDomain();
  // u runs the long way, v goes across the fillet.
  pFilletSrf->EvaluatePoint( sSrfDomain.Evaluate( 0,0 ), sSrfPt00 );
  pFilletSrf->EvaluatePoint( sSrfDomain.Evaluate( 1,0 ), sSrfPt10 );
  pFilletSrf->EvaluatePoint( sSrfDomain.Evaluate( 0,1 ), sSrfPt01 );
  pFilletSrf->EvaluatePoint( sSrfDomain.Evaluate( 1,1 ), sSrfPt11 );

  double dDist0 = sSrfPt00.DistanceBetween( sSrfPt10 );  // We'll use these again.
  double dDist1 = sSrfPt01.DistanceBetween( sSrfPt11 );

  if ( dDist0 < m_dThisApproxTol3d )
  {
      if ( dDist1 < m_dThisApproxTol3d )
      {
          return SM_SUCCESS;
      }
  }

  // Ok, the fillet surface does not meet itself smoothly, we have to do something.
  // Either it overlaps, or there is a gap.  Check that.
  // We can work from the fillet surface corner points: drop the fillet surface end point
  // to the pt/vector start point plus its u-derivative.
  SmVector3d sSu00, sSv00;
  double dT;
  pFilletSrf->Evaluate1stDerivatives( sSrfDomain.Evaluate(0,0), TRUE, TRUE, sSrfPt00, sSu00, sSv00 );
  SER( smgu_LineClosestPoint( sSrfPt00, sSu00, sSrfPt10, dT ) );

  // For convenience: collect all FilletVertexuses involved in the rails.

  if ( dT > 0.0 )
  {
      // Overlap: we have to split.
      // Find a FilletVertexuse at which to split.
      // The FVU is used only to get the FilletVertex, and which rail.
      // We could split at the low or high end.  Let's do the high end.
      // That way the main fillet is first, type DEFAULT, and the ROLLOVER
      // fillet will be the small gap geom.
      // A splitting FilletVertex will be interior to its base Face.
      // Look at all interior FVUs and take the farthest-back one, w.r.t the fillet surface.
      // For this we can sort against the same point/vector used above.

      SM_ASSERT( m_vVertices.GetSize() == 2 );

      SmFilletVertex *pFV = m_vVertices[0];
      // Looking for a vertex where a rail hits a side edge:
      if ( pFV->GetFilletVertexType() != SM_FV_RAIL_X_EDGEUSE )
      {
          pFV = m_vVertices[1];
          if ( pFV->GetFilletVertexType() != SM_FV_RAIL_X_EDGEUSE )
            { SER( SM_ERR ); }
      }


      // The FG has four FVUs:
      SmFilletVertexuse * pFVUs[4];
      pFVUs[0] = pOrigFG->GetRailVertexuse( 0,0 );
      pFVUs[1] = pOrigFG->GetRailVertexuse( 0,1 );
      pFVUs[2] = pOrigFG->GetRailVertexuse( 1,0 );
      pFVUs[3] = pOrigFG->GetRailVertexuse( 1,1 );

      // Sort these: their int-curve parameters should be set.
      SmTArray< double > sCrvParams  (4);
      SmTArray< ULONG  > sSortIndices(4);
      for ( ULONG ii=0; ii<4; ii++ )
      {
          double dIntParam = pFVUs[ii]->GetTsectPnt().m_dCurveParameter;
          if ( dIntParam < -SM_EFF_ZERO )
            SER( SM_ERR );  // If this happens, we'll have to figure out something else.

          smgu_AddSorted( dIntParam, ii, TRUE, sCrvParams, &sSortIndices );
      }

      // Ok, we have to make a new fillet face in between.

      // Split at the 2nd sorted parameter: that will make pOrigFG be the beginning
      // short piece of fillet, which is the rollover part, because it is currently
      // outside the domain of one of the base faces.  When we trace the short
      // rollover section, we will move the out-of-domain parameters by a period
      // so that they are within the domain.


      SmFilletVertex *pOppositeFV = NULL;

      SmFilletVertexuse *pThisFVUse = pFVUs[ sSortIndices[ 1 ] ];

      // Ok, here we have a fillet geom that needs to be split
      // at this FilletVertex (pThisFVUse).

      // Which end of the FilletGeom are we at?  0 for start, 1 for end.
      // We're declaring this, because we'll keep the small rollover part at the beginning.
      ULONG lWhichEnd = 0;

      // It makes a difference to the split routine whether the original
      // FG, or the new one, is a tangent-rollover.  The assumption is that
      // 'this' FG is the low end, and the new one is the high end; so,
      pOrigFG->SetFilletGeomType( SM_FG_TANGENT_ROLLOVER );

      SmFilletGeom *pNewFG = NULL;
      SmStatus eStat = pOrigFG->GetFilletSolver()->SplitFilletGeomAtSideEdge(
                  pOrigFG,
                  pThisFVUse,
                  lWhichEnd,
                  &pNewFG );

      // If failure, the whole fillet could still succeed, so don't SER.
      if ( eStat != SM_SUCCESS )
        { return SM_SUCCESS; }

#ifdef SM_DEBUG_CODE
SmBoolean bDebug_FVUs=FALSE;
      if ( bDebug_FVUs )
      {
        pOrigFG->DumpFilletVUs();
        pNewFG ->DumpFilletVUs();
      }
#endif // SM_DEBUG_CODE

      // After splitting, pOrigFG is the low end and pNewFG is the
      // high end: the newly created fillet vertices are at the high
      // end of pOrigFG and the low end of pNewFG.
      //
      // The fillet vertex at which we just split, pThisFV, is left at
      // either the low end of pOrigFG (if lWhichEnd == 0) or the
      // high end of pNewFG (otherwise).
      //
      // One of the new vertices now takes over for pThisFV, being the
      // intersection of the rail and the (tangent) side edge.
      // The split routine we just called takes care of that.
      //
      // The split fillet vertex, pThisFV, is now just a mate to the
      // corresponding FV on the other rail.  Deal with that.

      SmFilletVertex * pThisFV = SM_CAST_PTR( SmFilletVertex, pThisFVUse->GetVertex() );

      pThisFV->SetFilletVertexType( SM_FV_MATE );
      pThisFV->SetStatus( SM_FIL_UNPROCESSED );

      // Set it as mate of its opposite fillet vertex.
      SmFilletVertexuse *pOppositeFVU = NULL;

      // Grab the correct rail.
      ULONG lRailIdx;
      pOrigFG->GetRailIndexOfVertex( pThisFV, lRailIdx );
      SmFilletEdge *pThisRail = pOrigFG->GetRail( lRailIdx );

      if ( lWhichEnd == 0 )
          pOppositeFVU = pOrigFG->GetRailVertexuse( 1-lRailIdx, 0 );
      else
          pOppositeFVU = pNewFG->GetRailVertexuse( 1-lRailIdx, 1 );

      pOppositeFV = SM_CAST_PTR( SmFilletVertex, pOppositeFVU->GetVertex() );

      if ( pOppositeFV != NULL )
      {
          pThisFV->SetMate( 0, pOppositeFV );
          pOppositeFV->SetMate( 0, pThisFV );

          // Also tell it that its geometry is kaput
          // (because pThisFV is on a different base surface).
          pOppositeFV->SetStatus( SM_FIL_UNPROCESSED );
      }

      // Also, pThisFV is now in a face.
      SmFace *pNewBaseFace = pNewFG->GetRail( lRailIdx )->GetOriginalFace();
      pThisFV->SetPointClass( SM_PC_FACE, pNewBaseFace );

      // Edges are kaput too.
      for ( ULONG j = 0; j < m_vEdges.GetSize(); j++ )
        { m_vEdges[j]->SetStatus( SM_FIL_UNPROCESSED ); }

      // Also: there are now two new FilletVertex's, which belong
      // to this FilletCorner.  Add them to our list, and set
      // their back-pointer to us.
      // 'Same' is on the same side as pThisFV, 'Opp' as pOppositeFV.
      // Also recall that the new FilletVertex's are at the high end
      // of the original FG -- hence the 2nd arguments:

      SmFilletVertex * pNewFVSame = pOrigFG->GetRailVertex(   lRailIdx, 1 );
      SmFilletVertex * pNewFVOpp  = pOrigFG->GetRailVertex( 1-lRailIdx, 1 );

      // pNewFVSame should be all set, it's what pThisFV used to be.
      // But pNewFVOpp is now in a face.
      SmFace *pOppBaseFace = pOrigFG->GetRail( 1-lRailIdx )->GetOriginalFace();
      pNewFVOpp->SetPointClass( SM_PC_FACE, pOppBaseFace );

      // We also have to take responsibility for the new edge.
      SmTArray<SmEdge*> aNewEdges;
      pNewFVSame->GetCommonEdges( pNewFVOpp, aNewEdges );
      SM_ASSERT( aNewEdges.GetSize() == 1 );
      if ( aNewEdges.GetSize() > 0 )
      {
          SmFilletEdge *pThisFilEdge = SM_CAST_PTR( SmFilletEdge, aNewEdges[0] );
          this->m_vEdges.Add( pThisFilEdge );
          pThisFilEdge->SetFilletCorner( this );
      }

      // We have to update the fillet surfaces in the fillet geoms here.
      // A prerequisite to that is recalculating the vertex geom.

      this->CalcCornerVertGeom();

#ifdef SM_DEBUG_CODE
      if ( bDebug_FVUs )
      {
          pOrigFG->DumpFilletVUs();
          pNewFG ->DumpFilletVUs();
      }
#endif // SM_DEBUG_CODE


      // Adjust the uv positions in the TsectPnts in the Fillet Vertexuses for the seams.

      SmFilletSolver * pFS = pOrigFG->GetFilletSolver();
      SM_ASSERT( pNewFG->GetFilletSolver() == pFS );

      SmOffsetSurface * pSurf1 = pFS->GetSurface(0);
      SmOffsetSurface * pSurf2 = pFS->GetSurface(1);
      const SmSurface *pBaseSurf1 = pSurf1->GetBaseSurface();
      const SmSurface *pBaseSurf2 = pSurf2->GetBaseSurface();
      SmExtent2d sDomain1 = pSurf1->GetNaturalUVDomain();
      SmExtent2d sDomain2 = pSurf2->GetNaturalUVDomain();

      // Note, surface closure is relatively expensive to calculate.
      // If we need it (for seam checking), we'll calculate it once and pass flags.
      SmSurfParamType eSurfClosure1 = sm_SetSurfaceClosure( pSurf1, sDomain1 );
      SmSurfParamType eSurfClosure2 = sm_SetSurfaceClosure( pSurf2, sDomain2 );

      // Process pOrig FG.
      SmTArray<SmTsectPnt*> sTsectPnts(2);

      // Also keep track of which FVUs these come from.
      SmTArray< SmFilletVertexuse* > sUsedFVUsOrig;

      for ( ULONG ii=0; ii<2; ii++ )
      {
          for ( ULONG jj=0; jj<2; jj++ )
          {
              SmFilletVertexuse *pFVU = pOrigFG->GetRailVertexuse( ii, jj );

              SM_ASSERT( pFVU != NULL ); // This would be a problem.
              if ( pFVU == NULL )
                { continue; }

              SmFilletVertex * pFVertex = (SmFilletVertex*) ( pFVU->GetVertex() );
              if (pFVertex->GetStatus() == SM_FIL_UNPROCESSED ||
                  pFVertex->GetFilletVertexType() == SM_FV_MATE ||
                  pFVertex->GetFilletVertexType() == SM_FV_RAIL_X_EXTENDED_EDGEUSE)
                    { continue; }

              SmTsectPnt & rTsectPnt = pFVU->GetTsectPnt();
              sTsectPnts.Add( &rTsectPnt );

              // Also keep track of which FVU this came from.
              sUsedFVUsOrig.Add( pFVU );
          }
      } // end collecting FilletVertexuses

      // There should be two of them.
      SM_ASSERT( sTsectPnts.GetSize() == 2 );

      // Map any out-of-domain uv points back in, with a periodic shift.
      for ( ULONG ii=0; ii<sTsectPnts.GetSize(); ii++ )
      {
          SmTsectPnt * rTsectPnt = sTsectPnts[ii];
          SmPoint2d sUV = rTsectPnt->UVPos(0);
          if ( sm_CheckSeamOut( pBaseSurf1, sUV, m_dThisApproxTol3d , eSurfClosure1 ) )
          {
              rTsectPnt->UVPos(0) = sUV;
          }

          sUV = rTsectPnt->UVPos(1);
          if ( sm_CheckSeamOut( pBaseSurf2, sUV, m_dThisApproxTol3d , eSurfClosure2 ) )
          {
              rTsectPnt->UVPos(1) = sUV;
          }
      }


      // Now process pNewFG.
      sTsectPnts.ReSet();
      SmTArray< SmFilletVertexuse* > sUsedFVUsNew;

      for ( ULONG ii=0; ii<2; ii++ )
      {
          for ( ULONG jj=0; jj<2; jj++ )
          {
              SmFilletVertexuse *pFVU = pNewFG->GetRailVertexuse( ii, jj );
              SM_ASSERT( pFVU != NULL ); // This would be a problem.
              if ( pFVU == NULL )
                { continue; }

              SmFilletVertex * pFVertex = (SmFilletVertex*) (pFVU->GetVertex());
              if (pFVertex->GetStatus() == SM_FIL_UNPROCESSED ||
                  pFVertex->GetFilletVertexType() == SM_FV_MATE ||
                  pFVertex->GetFilletVertexType() == SM_FV_RAIL_X_EXTENDED_EDGEUSE)
                    { continue; }

              SmTsectPnt & rTsectPnt = pFVU->GetTsectPnt();
              sTsectPnts.Add( &rTsectPnt );

              // Also keep track of which FVU this came from.
              sUsedFVUsNew.Add( pFVU );
          }
      } // end collecting FilletVertexuses

      // There should be two of them.
      SM_ASSERT( sTsectPnts.GetSize() == 2 );

      // Map any out-of-domain uv points back in, with a periodic shift.
      for ( ULONG ii=0; ii<sTsectPnts.GetSize(); ii++ )
      {
          SmTsectPnt * rTsectPnt = sTsectPnts[ii];
          SmPoint2d sUV = rTsectPnt->UVPos(0);
          if ( sm_CheckSeamOut( pBaseSurf1, sUV, m_dThisApproxTol3d , eSurfClosure1 ) )
          {
              rTsectPnt->UVPos(0) = sUV;
          }
          sUV = rTsectPnt->UVPos(1);
          if ( sm_CheckSeamOut( pBaseSurf2, sUV, m_dThisApproxTol3d, eSurfClosure2 ) )
          {
              rTsectPnt->UVPos(1) = sUV;
          }
      }

#ifdef SM_DEBUG_CODE
      if ( bDebug_FVUs )
      {
          pOrigFG->DumpFilletVUs();
          pNewFG ->DumpFilletVUs();
      }
#endif // SM_DEBUG_CODE

      // re-Calc Vert Geom with the modified UV values in the FVUs' TsectPnts.
      // First have to mark the changed ones UnProcessed.
      for ( ULONG ii=0; ii<sUsedFVUsOrig.GetSize(); ii++ )
        { ( (SmFilletVertex*)( sUsedFVUsOrig[ii]->GetVertex() ))->SetStatus( SM_FIL_UNPROCESSED ); }
      for ( ULONG ii=0; ii<sUsedFVUsNew.GetSize(); ii++ )
        { ( (SmFilletVertex*)( sUsedFVUsNew[ii]->GetVertex() ))->SetStatus( SM_FIL_UNPROCESSED ); }

      this->CalcCornerVertGeom();

#ifdef SM_DEBUG_CODE
      if ( bDebug_FVUs )
      {
          pOrigFG->DumpFilletVUs();
          pNewFG ->DumpFilletVUs();
      }
#endif // SM_DEBUG_CODE



      // We need a direction for ReCalcFilletGeom.
      // We are assuming that the split is very close to the beginning
      // of the smoothly-closed rail, so just use the start parameter.
      SmVector3d sPtDer[2];
      SmExtent1d sRailIvl = pThisRail->GetInterval();
      pThisRail->GetCurve()->Evaluate( sRailIvl.GetMin(), 1, FALSE, sPtDer );


      // Possible enhancement: We could avoid one ReCalcFilletGeom call.
      // The FG piece that's on the original two base faces could
      // just be trimmed, instead of recalculated.
      // If lWhichEnd is 1, then we could just trim pOrigFG.
      // Also, if lWhichEnd is 0, we could move the geometry from
      // pOrigFG into pNewFG, then just trim pNewFG,
      // and recalc pOrigFG.

      // Tell FilletSolver to use surfaces in the FilletGeoms.
      // First back up what's there, if anything, to restore when done.
      SmFilletGeom *pBackupFG = pFS->GetFilletGeomForSurfaces();

      pFS->SetFilletGeomForSurfaces( pOrigFG );
      pOrigFG->ReCalcFilletGeom( FALSE, &( sPtDer[1] ) );

      // Now the other FG
      pFS->SetFilletGeomForSurfaces( pNewFG );
      pNewFG->ReCalcFilletGeom( FALSE, &( sPtDer[1] ) );

      pFS->SetFilletGeomForSurfaces( pBackupFG );

  } // end if dT > 0 so have to split.

  return SM_SUCCESS;

} // end SmFilletNx1ClosedCorner::AdjustTangentCorner()

/*******************************************************************//**
PURPOSE: Adjust a corner for the Nx2 tangent case with N > 3.

NOTES:
    In that case, both rails are probably rail-int-edgeuse.
    If everything matches up just right geometrically,
    then we can mate the two fillet surfaces together,
    but there's a good chance that things don't meet properly,
    because the fillets are not rolling on the same face.
    In that case, we'll have to put another fillet surface
    between the two.

cbi: Plan: make a new Corner type, SM_FCR_TANGENT, subclass SmFilletTangentCorner.
     (possibly subtypes 3x2Tangent and Nx2Tangent.)
     Then this method will be SmFilletTangentCorner::AdjustTopology().
     Then this method won't have to check whether it's a Nx2 tangent corner;
     that check will be in Create().
cbi: try a 5x2 corner?
***********************************************************************/
SmStatus SmFilletNx2Corner::AdjustTangentCorner()
{
    // First check that this is the appropriate type.
    // Must be Nx2 tangent corner, and currently N must be 4.
    if ( m_pSolverEUs == NULL  ||  m_pSolverEUs->GetSize() != 2 ) { return SM_SUCCESS; }
    if ( m_pAllEUs    == NULL  ||  m_pAllEUs   ->GetSize() != 4 ) { return SM_SUCCESS; }

    // Check that the two filleted edges are tangent at this vertex.

    // Note: using NormalizedEvaluate() assumes that m_pSolverEUs go outward
    // from the filleted vertex.
    SmPoint3d sPt;
    SmVector3d s1stDeriv1, s1stDeriv2;
    (*m_pSolverEUs)[0]->NormalizedEvaluate( 0.0, FALSE, sPt, &s1stDeriv1 );   // TRUE = UV Eval, FALSE = 3d Eval
    (*m_pSolverEUs)[1]->NormalizedEvaluate( 0.0, FALSE, sPt, &s1stDeriv2 );   // TRUE = UV Eval, FALSE = 3d Eval

    double dAngTolDeg = 2.0; //cbi: some places use 5 degrees; sort that all out.
    if ( !s1stDeriv1.IsParallelTo( s1stDeriv2, dAngTolDeg)  )
        { return SM_SUCCESS; }


    // Ok, we are an Nx2-tangent corner.

    // loop on all of our FilletVertices
    //   looking for surface points being different.

    SmBoolean bCloseEnough = TRUE;

    // Locals
    SmTArray<SmFilletGeom*> sGeoms;

#ifdef SM_DEBUG_CODE
    int iDebugLevel = DebugLevel();
#endif // SM_DEBUG_CODE

    ULONG ii;
    for ( ii=0; ii<m_vVertices.GetSize(); ii++ )
    {
        SmFilletVertex * pAdjustVert = m_vVertices[ii];

        // Check whether this is the vertex that corresponds to the
        // rail hitting the side edge, and not the rails hitting
        // each other in a common face.
        if ( pAdjustVert->GetFilletVertexType() != SM_FV_RAIL_X_EDGEUSE )
          { continue; }

#ifdef SM_DEBUG_CODE
        if ( iDebugLevel > 0 ) {
            smgfx_SetLook(1,2, 1,0,0); pAdjustVert->Draw(); sm_GraphicsLoop();
            sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE

        // Ok, this fillet vertex corresponds to where the
        // rails from the two fillets hit a side edge.
        // Check whether those rails intersect the side edge at the same place.
        // In addition, the other end of this rib will have to coincide
        // in order for the fillet surfaces to mate here.

        pAdjustVert->GetFilletGeoms(sGeoms);
        if ( sGeoms.GetSize() != 2 )
            continue;  // SER( SM_ERR );  maybe already adjusted...

        SmFilletGeom * pFilletGeom1 = sGeoms[0];
        SmFilletGeom * pFilletGeom2 = sGeoms[1];

        //cbi: fix this in the future
        // We need the rail curves to work from.
        // If not there yet, just wait for a later call.
        if (   pFilletGeom1->GetRail(0)->GetCurve() == NULL
           ||  pFilletGeom1->GetRail(1)->GetCurve() == NULL
           ||  pFilletGeom2->GetRail(0)->GetCurve() == NULL
           ||  pFilletGeom2->GetRail(1)->GetCurve() == NULL )
        {
            return SM_SUCCESS;
        }

        // Get the vertexuses corresponding to the two surfaces,
        // and their TSectPnts:
        SmFilletVertexuse * pVU1 = pAdjustVert->GetVUAtRailEnd(pFilletGeom1);
        SmFilletVertexuse * pVU2 = pAdjustVert->GetVUAtRailEnd(pFilletGeom2, pVU1);
        SmTsectPnt &rIntPtSrf1 = pVU1->GetTsectPnt();
        SmTsectPnt &rIntPtSrf2 = pVU2->GetTsectPnt();

        SmSurface *pSrf1A = (SmSurface*)( rIntPtSrf1.m_apUserPointer[0] );
        SmSurface *pSrf1B = (SmSurface*)( rIntPtSrf1.m_apUserPointer[1] );
        SmSurface *pSrf2A = (SmSurface*)( rIntPtSrf2.m_apUserPointer[0] );
        SmSurface *pSrf2B = (SmSurface*)( rIntPtSrf2.m_apUserPointer[1] );

#ifdef SM_DEBUG_CODE
        if ( iDebugLevel > 0 ) {
            smgfx_SetLook(2,1, 0,0,1); pSrf1A->DrawUV(4,4,TRUE); sm_GraphicsLoop();
            smgfx_SetLook(2,1, 0,1,1); pSrf1B->DrawUV(4,4,TRUE); sm_GraphicsLoop();
            smgfx_SetLook(2,1, 1,0,0); pSrf2A->DrawUV(4,4,TRUE); sm_GraphicsLoop();
            smgfx_SetLook(2,1, 1,0,1); pSrf2B->DrawUV(4,4,TRUE); sm_GraphicsLoop();
            sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE
        SmBoolean bTmp0 = FALSE, bTmp1 = FALSE, bTmp2 = FALSE, bTmp3 = FALSE ; 
        SmBSplineSurface * pBSplineSurface0 = SM_CAST_PTR(SmBSplineSurface, pSrf1A) ;
        SmBSplineSurface * pBSplineSurface1 = SM_CAST_PTR(SmBSplineSurface, pSrf1B) ;
        SmBSplineSurface * pBSplineSurface2 = SM_CAST_PTR(SmBSplineSurface, pSrf2A) ;
        SmBSplineSurface * pBSplineSurface3 = SM_CAST_PTR(SmBSplineSurface, pSrf2B) ;
        SmTemporaryChangeValue<SmBoolean> sClean0(pBSplineSurface0 ? pBSplineSurface0->GetOutOfBoundsEnabled() : bTmp0, TRUE) ;
        SmTemporaryChangeValue<SmBoolean> sClean1(pBSplineSurface1 ? pBSplineSurface1->GetOutOfBoundsEnabled() : bTmp1, TRUE) ;
        SmTemporaryChangeValue<SmBoolean> sClean2(pBSplineSurface2 ? pBSplineSurface2->GetOutOfBoundsEnabled() : bTmp2, TRUE) ;
        SmTemporaryChangeValue<SmBoolean> sClean3(pBSplineSurface3 ? pBSplineSurface3->GetOutOfBoundsEnabled() : bTmp3, TRUE) ;

        SmPoint3d sSrf1Pt, sSrf2Pt;
        pSrf1A->EvaluatePoint( rIntPtSrf1.UVPos( 0 ), sSrf1Pt );
        pSrf2A->EvaluatePoint( rIntPtSrf2.UVPos( 0 ), sSrf2Pt );

#ifdef SM_DEBUG_CODE
        if ( iDebugLevel > 0 ) {
            smgfx_SetLook(1,6, 0,0,1);
            sSrf1Pt.Draw(); sm_GraphicsLoop();
            sSrf2Pt.Draw(); sm_GraphicsLoop();
            sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE

        if ( sSrf1Pt.DistanceBetween( sSrf2Pt ) > m_dThisApproxTol3d )
        {
            bCloseEnough = FALSE;
            break;
        }

        // The mates must agree as well for 4x2
        pSrf1B->EvaluatePoint( rIntPtSrf1.UVPos( 1 ), sSrf1Pt );
        pSrf2B->EvaluatePoint( rIntPtSrf2.UVPos( 1 ), sSrf2Pt );

#ifdef SM_DEBUG_CODE
        if ( iDebugLevel > 0 ) {
            smgfx_SetLook(1,6, 0,1,0);
            sSrf1Pt.Draw(); sm_GraphicsLoop();
            sSrf2Pt.Draw(); sm_GraphicsLoop();
            sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE

        if ( sSrf1Pt.DistanceBetween( sSrf2Pt ) > m_dThisApproxTol3d )
        {
            bCloseEnough = FALSE;
            break;
        }
    
    } // end of loop on all of our FilletVertices
      //   looking for surface points being different.

    if ( bCloseEnough )
    {
        return SM_SUCCESS;
    }


    // Ok, we have to make a new fillet face in between.

    // Loop over all of our side vertices again, splitting any
    // fillet geom that the vertex is in the middle of.
    ULONG lWhichFV, lWhichFilletGeom;

    // When we process a vertex, we don't want to process its mate.
    // Create a separate list so we can remove what we don't want to process.
    SmTArray<SmFilletVertex*> apFVsToProcess( m_vVertices );

    for ( lWhichFV = 0; lWhichFV < apFVsToProcess.GetSize(); lWhichFV++ )
    {
        SmFilletVertex * pThisFV = apFVsToProcess[lWhichFV];

        // Looking for a vertex where a rail hits a side edge:
        if ( pThisFV->GetFilletVertexType() != SM_FV_RAIL_X_EDGEUSE )
            continue;

        pThisFV->GetFilletGeoms( sGeoms );
        SM_ASSERT( sGeoms.GetSize() == 2 );
        if ( sGeoms.GetSize() != 2 )
            { continue; }

        // This Fillet Vertex has two Fillet Vertexuses, one on each FilletGeom.
        // If either is interior to its FilletGeom, split that fillet geom.
        SmFilletVertexuse * apVUs[2];
        apVUs[0] = pThisFV->GetVUAtRailEnd( sGeoms[0] );
        apVUs[1] = pThisFV->GetVUAtRailEnd( sGeoms[1], apVUs[0] );

        // Drop the vertex position to the corresponding rails in
        // each FilletGeom.  If it's internal to the rail,
        // split the FilletGeom.

        SmPoint3d sVertexPos = pThisFV->GetPoint();

        // This is something we'll need after the next loop:
        SmFilletVertex *pOppositeFV = NULL;

        // Look at both FilletGeoms
        for ( lWhichFilletGeom = 0; lWhichFilletGeom <2; lWhichFilletGeom++ )
        {
            SmFilletGeom *pOrigGeom = sGeoms[ lWhichFilletGeom ];

            SmFilletVertexuse *pThisFVUse = apVUs[ lWhichFilletGeom ];

            // Grap the correct rail
            ULONG lRailIdx;
            pOrigGeom->GetRailIndexOfVertex( pThisFV, lRailIdx );
            SmFilletEdge *pThisRail = pOrigGeom->GetRail( lRailIdx );

            SmExtent1d sRailIvl = pThisRail->GetInterval();
            double *pdGuessParam = NULL; //cbi.
            SmBoolean bSuccess;
            double dSplitParam = 0.0;
            double dDist = 0.0;

            pThisRail->GetCurve()->DropPoint(sRailIvl,         // in : target curve allowed domain
                                             sVertexPos,       // in : Point to drop to curve
                                             NULL,             // in : For Snapping, NULL=NoSnap, Vec pointing to CrvInside from desired EndPt for snapping.
                                                               //      when DropPt is within dDistTol in cpVecToCrvInside dir of EndPt, Snap to Ivl EndPt.
                                                               //      Vec handles ambiguities and makes sure that curves are not just touching at the ends.
                                             SM_BIG_DOUBLE,    // in : Used for SnapDist, and SM_SO_INTERSECT sXSectTol3d distance.
                                                               //      sXSectTol3d = dDistTol ;      SM_SO_INTERSECT 
                                                               //      sXSectTol3d = SM_BIG_DOUBLE ; SM_SO_MINIMIZE, SM_SO_NORMALIZE
                                                               //      sSnapTol3d  = dDistTol ;      SM_SO_MINIMIZE, SM_SO_NORMALIZE, SM_SO_INTERSECT    
                                             pdGuessParam,     // in : If given, call cheap LocalPointSolve() else call expensive GlobalPointSolve()
                                             bSuccess,         // out: TRUE = found a drop point
                                             dSplitParam,      // out: found drop curve param
                                             dDist,            // out: found drop distance
                                             SM_SO_MINIMIZE) ; // in : SM_SO_MINIMIZE = allow nonNormal drops near endPoints
                                                               //      SM_SO_INTERSECT= like Minimize but point must be within dDistTol
                                                               //      SM_SO_NORMALIZE= exclude nonNormal drops near endPoints
                                                               //      default:[SM_SO_MINIMIZE] to preserve original behavior

            if ( !bSuccess )
                { continue; }

            if ( sRailIvl.IsValueOnBoundary( dSplitParam, m_dThisApproxTol3d ) )
            {
                continue;
            }

            // Ok, here we have a fillet geom that needs to be split
            // at this FilletVertex (pThisFVUse).

            // Which end of the FilletGeom are we at?  0 for start, 1 for end.
            ULONG lWhichEnd = 0;
            if ( pOrigGeom->GetRailVertexuse( lRailIdx, 1 ) == pThisFVUse )
              { lWhichEnd = 1;  }

            // It makes a difference to the split routine whether the original
            // FG, or the new one, is a tangent-rollover.  The assumptions are:
            // 1: 'this' FG is the low end, and the new one is the high end; so,
            // 2: if we're moving from our base surfaces onto the adjacent face,
            //    then the new FG will be TangentRollover, otherwise the
            //    original FG is TangentRollover.
            // So we set the type of pOrigGeom according to what the low-end
            // part will be.
            // So, (only) if the low end is being changed, we change
            // the original FG to TangentRollover.
            if ( lWhichEnd == 0 )
                { pOrigGeom->SetFilletGeomType( SM_FG_TANGENT_ROLLOVER ); }


            SmFilletGeom *pNewGeom = NULL;
            SmStatus eStat =
                pOrigGeom->GetFilletSolver()->SplitFilletGeomAtSideEdge(
                    pOrigGeom,
                    pThisFVUse,
                    lWhichEnd,
                    &pNewGeom );

            // If failure, the whole fillet could still succeed, so don't SER.
            if ( eStat != SM_SUCCESS )
              { continue; }


            // After splitting, pOrigGeom is the low end and pNewGeom is the
            // high end: the newly created fillet vertices are at the high
            // end of pOrigGeom and the low end of pNewGeom.
            //
            // The fillet vertex at which we just split, pThisFV, is left at
            // either the low end of pOrigGeom (if lWhichEnd == 0) or the
            // high end of pNewGeom (otherwise).
            //
            // One of the new vertices now takes over for pThisFV, being the
            // intersection of the rail and the (tangent) side edge.
            // The split routine we just called takes care of that.
            //
            // The split fillet vertex, pThisFV, is now just a mate to the
            // corresponding FV on the other rail.  Deal with that.

            pThisFV->SetFilletVertexType( SM_FV_MATE );
            pThisFV->SetStatus( SM_FIL_UNPROCESSED );

            // Set it as mate of its opposite fillet vertex.
            SmFilletVertexuse *pOppositeFVU = NULL;

            if ( lWhichEnd == 0 )
                pOppositeFVU = pOrigGeom->GetRailVertexuse( 1-lRailIdx, 0 );
            else
                pOppositeFVU = pNewGeom->GetRailVertexuse( 1-lRailIdx, 1 );

            pOppositeFV = SM_CAST_PTR( SmFilletVertex, pOppositeFVU->GetVertex() );

            if ( pOppositeFV != NULL )
            {
                pThisFV->SetMate( 0, pOppositeFV );
                pOppositeFV->SetMate( 0, pThisFV );

                // Also tell it that its geometry is kaput
                // (because pThisFV is on a different base surface).
                pOppositeFV->SetStatus( SM_FIL_UNPROCESSED );
            }

            // Also, pThisFV is now in a face.
            SmFace *pNewBaseFace = pNewGeom->GetRail( lRailIdx )->GetOriginalFace();
            pThisFV->SetPointClass( SM_PC_FACE, pNewBaseFace );

            // Edges are kaput too.
            for ( ULONG j = 0; j < m_vEdges.GetSize(); j++ )
              { m_vEdges[j]->SetStatus( SM_FIL_UNPROCESSED ); }

            // Also: there are now two new FilletVertex's, which belong
            // to this FilletCorner.  Add them to our list, and set
            // their back-pointer to us.
            // 'Same' is on the same side as pThisFV, 'Opp' as pOppositeFV.
            // Also recall that the new FilletVertex's are at the high end
            // of the original FG -- hence the 2nd arguments:

            SmFilletVertex * pNewFVSame = pOrigGeom->GetRailVertex(   lRailIdx, 1 );
            SmFilletVertex * pNewFVOpp  = pOrigGeom->GetRailVertex( 1-lRailIdx, 1 );

            // pNewFVSame should be all set, it's what pThisFV used to be.
            // But pNewFVOpp is now in a face.
            SmFace *pOppBaseFace = pOrigGeom->GetRail( 1-lRailIdx )->GetOriginalFace();
            pNewFVOpp->SetPointClass( SM_PC_FACE, pOppBaseFace );

            // We also have to take responsibility for the new edge.
            SmTArray<SmEdge*> aNewEdges;
            pNewFVSame->GetCommonEdges( pNewFVOpp, aNewEdges );
            SM_ASSERT( aNewEdges.GetSize() == 1 );
            if ( aNewEdges.GetSize() > 0 )
            {
                SmFilletEdge *pThisFilEdge = SM_CAST_PTR( SmFilletEdge, aNewEdges[0] );
                this->m_vEdges.Add( pThisFilEdge );
                pThisFilEdge->SetFilletCorner( this );
            }

            // We have to update the fillet surfaces in the fillet geoms here.
            // A prerequisite to that is recalculating the vertex geom.

            this->CalcCornerVertGeom();


            // We need a direction for ReCalcFilletGeom.
            // We already dropped this vertex to the rail being split,
            // so just evaluate the rail's tangent direction there.

            SmVector3d sPtDer[2]; // point and 1st derivative
            pThisRail->GetCurve()->Evaluate( dSplitParam, 1, FALSE, sPtDer );


            // Possible enhancement: We could avoid one ReCalcFilletGeom call.
            // The FG piece that's on the original two base faces could
            // just be trimmed, instead of recalculated.
            // If lWhichEnd is 1, then we could just trim pOrigGeom.
            // Also, if lWhichEnd is 0, we could move the geometry from
            // pOrigGeom into pNewGeom, then just trim pNewGeom,
            // and recalc pOrigGeom.

//cbi: this SetFilletGeomForSurfaces(): do it right in ReCalc?
            // Tell FilletSolver to use surfaces in the FilletGeoms.
            // First back up what's there, if anything, to restore when done.
            SmFilletSolver *pFS = pOrigGeom->GetFilletSolver();
            SmFilletGeom *pBackupFG = pFS->GetFilletGeomForSurfaces();

            pFS->SetFilletGeomForSurfaces( pOrigGeom );
            pOrigGeom->ReCalcFilletGeom( FALSE, &( sPtDer[1] ) );

            // Now the other FG
            pFS->SetFilletGeomForSurfaces( pNewGeom );
            pNewGeom->ReCalcFilletGeom( FALSE, &( sPtDer[1] ) );

            pFS->SetFilletGeomForSurfaces( pBackupFG );


            // Now, the other FilletGeom, the one that didn't get split,
            // needs to be trimmed back to the far end of the split FG.
            SmFilletGeom *pOtherGeom = sGeoms[ 1-lWhichFilletGeom ];

            // Grap the correct rail
            ULONG lOtherRailIdx;
            pOtherGeom->GetRailIndexOfVertex( pThisFV, lOtherRailIdx );
            SmFilletEdge *pOtherRail = pOtherGeom->GetRail( lOtherRailIdx );

            // The fillet vertex at which the other FG gets trimmed to
            // is still pointed to by pThisFV.  (See earlier comments.)
            SmPoint3d sTrimPt = pThisFV->GetPoint();

            SmExtent1d sOtherRailIvl = pOtherRail->GetInterval();

            double dOtherSplitParam = 0;
            pOtherRail->GetCurve()->DropPoint(sOtherRailIvl,    // in : target curve allowed domain
                                              sTrimPt,          // in : Point to drop to curve
                                              NULL,             // in : For Snapping, NULL=NoSnap, Vec pointing to CrvInside from desired EndPt for snapping.
                                                                //      when DropPt is within dDistTol in cpVecToCrvInside dir of EndPt, Snap to Ivl EndPt.
                                                                //      Vec handles ambiguities and makes sure that curves are not just touching at the ends.
                                              SM_BIG_DOUBLE,    // in : Used for SnapDist, and SM_SO_INTERSECT sXSectTol3d distance.
                                                                //      sXSectTol3d = dDistTol ;      SM_SO_INTERSECT 
                                                                //      sXSectTol3d = SM_BIG_DOUBLE ; SM_SO_MINIMIZE, SM_SO_NORMALIZE
                                                                //      sSnapTol3d  = dDistTol ;      SM_SO_MINIMIZE, SM_SO_NORMALIZE, SM_SO_INTERSECT    
                                              pdGuessParam,     // in : If given, call cheap LocalPointSolve() else call expensive GlobalPointSolve()
                                              bSuccess,         // out: TRUE = found a drop point
                                              dOtherSplitParam, // out: found drop curve param
                                              dDist,            // out: found drop distance
                                              SM_SO_MINIMIZE) ; // in : SM_SO_MINIMIZE = allow nonNormal drops near endPoints
                                                                //      SM_SO_INTERSECT= like Minimize but point must be within dDistTol
                                                                //      SM_SO_NORMALIZE= exclude nonNormal drops near endPoints
                                                                //      default:[SM_SO_MINIMIZE] to preserve original behavior

            if ( !bSuccess )
                { continue; }

            // Low or high end of other FG?
            SmFilletVertexuse *pOtherFVUse = apVUs[ 1-lWhichFilletGeom ];

            ULONG lOtherWhichEnd = 0;
            if ( pOtherGeom->GetRailVertexuse( lRailIdx, 1 ) == pOtherFVUse )
                lOtherWhichEnd = 1;

            // Trim sOtherRailIvl accordingly
            if ( lOtherWhichEnd == 0 )
            {
                sOtherRailIvl.SetMinMax( dOtherSplitParam, sOtherRailIvl.GetMax() );
            }
            else
            {
                sOtherRailIvl.SetMinMax( sOtherRailIvl.GetMin(), dOtherSplitParam );
            }

            // Finally, do the trim.
            pOtherGeom->Trim( sOtherRailIvl ); // may snap sIvl by tol to existing knots

        }  // end loop over two FilletGeom's of this FilletVert.

        // Remove the opposite-rail FilletVertex from the list to be processed.
        // so that we don't try to split the other FilletGeom at it.
        ULONG lIdx;
        if (    pOppositeFV != NULL
             && apFVsToProcess.FindElement( pOppositeFV, lIdx ) )
        {
            apFVsToProcess.RemoveAt( lIdx );
            if ( lIdx <= lWhichFV )  // if we removed one before this one,
                lWhichFV--;          //   slide us back one.
        }
    }  // end loop over all FilletVert's.


    return SM_SUCCESS;
} // end SmFilletNx2Corner::AdjustTangentCorner

/*******************************************************************//**
PURPOSE: Adjust the corner topology for the bevel cases where two fillets
    of the corner intersect the side edge at different locations.
    One additional edge will be added to corner to bound the fillet
    that hits farther away from the filleted vertex.

NOTES:
***********************************************************************/
SmStatus SmFilletNx2Corner::AdjustTopology()
{
#ifdef SM_DEBUG_CODE
  if ( DebugLevel() > 0 ) 
    {
      smgfx_SetLook(2,4, 0,0,1); this->Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // Can't do it if more than four total edges
  if(m_pAllEUs->GetSize() > 4)
    { SER( SM_ERR ) ; }

  // Check for mixed-convexity case: special routine for that.
  // For the 3x2 mixed-cvxty case, we create a 3-sided face.
  // We have created the edges and vertices for it,
  // so this is how we check for that case:

  ULONG lNumVertexEdges = m_vEdges.GetSize();
  if ( lNumVertexEdges == 3 )
    {   // Mixed-convexity corners
      SER( AdjustConcaveCornerTopology() );
      return SM_SUCCESS;
    }

  // Note: do this separately:
  // // Another special case is the Nx2 tangent case, with N > 3.
  // // In that case we might have to put another fillet surface
  // // in between the two that are there now.
  // if ( this->IsBlendingCorner()  &&  m_pAllEUs->GetSize() > 3 )
  //   {
  //     SER( AdjustTangentCorner() );
  //     return SM_SUCCESS;
  //   }

  // Examine the vertices that have been created for this corner.
  // Typically there will be two, one where the rails from the two
  // fillets intersect in the common face (between filleted edges),
  // and one where the other rail of each fillet intersects the
  // side edge (the un-filleted edge of this corner).
  // If there's only one side edge between their RAIL_X_EDGEUSE rails,
  // and the fillets are the same size, and things are generally square, 
  // then their rails will hit the side edge at the same point.
  // But, if these intersections are not the same point, then we have
  // to split the vertex into two, and insert a new edge to connect them.

  ULONG ii;
  for(ii=0;ii<m_vVertices.GetSize();ii++)
    {
      SmFilletVertex * pAdjustVert = m_vVertices[ii];

#ifdef SM_DEBUG_CODE
      if ( DebugLevel() > 0 ) 
        {
          smgfx_SetLook(1,8, 1,0,0); pAdjustVert->Draw(); sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE

      // Check whether this is the vertex that corresponds to the
      // rail hitting the side edge, and not the rails hitting
      // each other in the common face.
      if(pAdjustVert->GetFilletVertexType() != SM_FV_RAIL_X_EDGEUSE)
        { continue ; }

      // Ok, this is the new fillet vertex corresponding to where the
      // rails from the two fillets are supposed to hit the side edge.
      // Check whether those rails intersect the side edge at the same place.

      SmTArray<SmFilletGeom*> sGeoms;
      pAdjustVert->GetFilletGeoms(sGeoms);
      if(sGeoms.GetSize() != 2)
        { continue ; }  // SER( SM_ERR );  maybe already adjusted...

      SmFilletGeom      * pFilletGeom1   = sGeoms[0] ;
      SmFilletGeom      * pFilletGeom2   = sGeoms[1] ;
      SmEdgeuse         * pSideEU1       = (SmEdgeuse*)pAdjustVert->GetPointClassObject() ;
      SmFilletVertexuse * pVU1           = pAdjustVert->GetVUAtRailEnd(pFilletGeom1) ;
      SmFilletVertexuse * pVU2           = pAdjustVert->GetVUAtRailEnd(pFilletGeom2, pVU1) ;
      double              dEdgeParam1    = pVU1->GetTsectPnt().m_dCurveParameter ;
      double              dEdgeParam2    = pVU2->GetTsectPnt().m_dCurveParameter ;
      SmPoint3d           sPnt1, sPnt2 ;  
      SmCurve           * pSideEdgeCurve = pSideEU1->GetEdge()->GetCurve() ;

      pSideEdgeCurve->EvaluatePoint(dEdgeParam1,sPnt1) ;
      pSideEdgeCurve->EvaluatePoint(dEdgeParam2,sPnt2) ;

      double dDist = sPnt1.DistanceBetween(sPnt2) ;

      // Check for the big-radius case, where we're rolling a large fillet
      // on the inside of a smaller-radius edge.
      // In that case, we'll create a 3x3 corner for the vertex.
      //
      // We'll also check for the Nx2 tangent case, with N > 3.
      if(this->IsBlendingCorner())     // Big-radius doesn't apply if Beveled.
        {
          SmBoolean           bConvertedToCorner;
          SmFilletExecutive * pExec = GetFilletExecutive();

          // This routine in the fillet exec will check for the
          // large-radius case, and convert the corner if so.
          SER( pExec->TestAndConvertBigRadFillet(pFilletGeom1, bConvertedToCorner ));
          if ( !bConvertedToCorner )
            {
              SER( pExec->TestAndConvertBigRadFillet( pFilletGeom2,
                      bConvertedToCorner ));
            }

          if ( bConvertedToCorner ) 
            { return SM_SUCCESS ; }

          if(dDist > 500.0 * m_dThisApproxTol3d)
            { MSG( _T("Big gap found. Will try to fix it later") ) ; }

          continue ;

        }  // End big-radius check.

      SM_ASSERT(m_bBevel == TRUE); // Bevel cases

      // If rails hit side edge at same point, no adjustment is needed.
      if ( dDist < m_dThisApproxTol3d )
        { continue ; }

      // Ok, if we're here, we're going to adjust the topology
      // of this corner.  We're going to make two separate vertices
      // where the rails from each of the two fillets hit the common
      // side edge, and a little edge between them.

      m_bTopologyAdjusted = TRUE;

      // Note on naming: these aren't necessarily smaller and larger radii.
      // For a given radius, the sharper the interior angle of the edge,
      // the farther away from the vertex the intersection will be.
      // But these names are descriptive, they make it easy to tell
      // which we're talking about: pSmallRadGeom means the one that
      // hits closer to the vertex.

      SmPoint3d      sVertPnt      = m_cpVertex->GetPoint() ;
      SmFilletGeom * pSmallRadGeom = NULL ;
      SmFilletGeom * pLargeRadGeom = NULL ;
      SmVertexuse  * pVUOnSideFace = NULL ;
      SmFace       * pSideFace     = NULL ;
      SmPoint3d      sPntOnSideFace ;

      // We'll split this vertex and keep the farther point (along the edge)
      // as the one SM_FV_RAIL_X_EDGEUSE.  The new vertex will be on
      // the side face and will be labeled as SM_FV_FILLET_X_FILLET
      if(sPnt1.DistanceBetween(sVertPnt) < sPnt2.DistanceBetween(sVertPnt))
        {
          pAdjustVert->SetPoint( sPnt2 ) ;
          pAdjustVert->SetOriginalTParam( dEdgeParam2 ) ;

          pSideFace      = pSideEU1->GetFace() ;
          pVUOnSideFace  = pVU1 ;
          pSmallRadGeom  = pFilletGeom1 ;
          pLargeRadGeom  = pFilletGeom2 ;
          sPntOnSideFace = sPnt1 ;
        } // end sPnt2 further away from VertPnt branch
      else // sPnt1 further away from VertPnt branch
        {
          pAdjustVert->SetPoint( sPnt1 ) ;
          pAdjustVert->SetOriginalTParam( dEdgeParam1 ) ;

          pSideFace      = pSideEU1->GetRadial()->GetFace() ;
          pVUOnSideFace  = pVU2 ;
          pSmallRadGeom  = pFilletGeom2 ;
          pLargeRadGeom  = pFilletGeom1 ;
          sPntOnSideFace = sPnt2 ;
        } // end sPnt1 further away from VertPnt branch

      // The vert that is farther from the filleted vertex, which is
      // the one where one rail hits the side edge, is now done.
      // Label it as 'processed'.
      pAdjustVert->SetStatus( SM_FIL_PROCESSED );

      SmFilletVertex * pVertOnSideEdge = pAdjustVert;

      // Create new fillet vertex on the 'side' face
      SmFilletVertex * pVertOnSideFace = new (m_pPseudoBrep) SmFilletVertex(this) ; NER( pVertOnSideFace );
      SmObjDelete sClean1( pVertOnSideFace );

      pVertOnSideFace->SetPoint( sPntOnSideFace );
      pVertOnSideFace->SetPointClass( SM_PC_FACE, pSideFace );
      pVertOnSideFace->SetFilletVertexType( SM_FV_FILLET_X_FILLET );

      SmBoolean        bInsertAfter = TRUE;
      ULONG            lIndex       = ii+1;
      SmFilletVertex * pOtherVert   = NULL;

      if(lIndex < m_vVertices.GetSize() )
        {
          pOtherVert = m_vVertices[lIndex];
        }
      else
        {
          // Insert new vertex before i
          lIndex--;
          bInsertAfter = FALSE;
          pOtherVert   = m_vVertices[ lIndex-1 ];
        }

      m_vVertices.InsertAt( lIndex, pVertOnSideFace );

      // Find the vertexuse of rail which is on the
      // 'side' face and reconnect it to the newly created vertex.
      pVertOnSideEdge->Remove( pVUOnSideFace );
      SER(pVertOnSideFace->PostInsert( pVUOnSideFace ));

      // Recreate fillet edges
      SmFilletEdge * pFilletEdge1 = NULL;
      if ( bInsertAfter )
        {
          SER( SmFilletEdge::MakeFilletEdge( m_pPseudoBrep,   // in : target Brep to receive new topology objects
                                             pVertOnSideEdge, // in : start of new FilletEdge
                                             pVertOnSideFace, // in : end   of new FilletEdge
                                             pFilletEdge1,    // out: newly allocated FilletEdge - no Curve or UVTrimCurve data
                                             this ));         // in : NewFilletEdge's filletCorner, gets stored in rpNewFilletEdge->m_cpCorner
                                                              //      NULL to ignore, default:[NULL]
        }
      else
        {
          SER( SmFilletEdge::MakeFilletEdge( m_pPseudoBrep,   // in : target Brep to receive new topology objects
                                             pVertOnSideFace, // in : start of new FilletEdge
                                             pVertOnSideEdge, // in : end   of new FilletEdge
                                             pFilletEdge1,    // out: newly allocated FilletEdge - no Curve or UVTrimCurve data
                                             this ));         // in : NewFilletEdge's filletCorner, gets stored in rpNewFilletEdge->m_cpCorner
                                                              //      NULL to ignore, default:[NULL]
        }
      pFilletEdge1->SetFilletEdgeType( SM_FE_FILLET_X_SIDE_FACE ) ;

      SmFilletEdgeuse * pPrimEU = (SmFilletEdgeuse*)pFilletEdge1->GetPrimaryEdgeuse() ;
      pPrimEU->SetFilletGeom( pLargeRadGeom );
      pFilletEdge1->SetOriginalFace( pSideFace );

      SmFilletEdge * pFilletEdge2 = NULL;
      if ( bInsertAfter )
        {
          SER( SmFilletEdge::MakeFilletEdge( m_pPseudoBrep,    // in : target Brep to receive new topology objects
                                             pVertOnSideFace,  // in : start of new FilletEdge
                                             pOtherVert,       // in : end   of new FilletEdge
                                             pFilletEdge2,     // out: newly allocated FilletEdge - no Curve or UVTrimCurve data
                                             this ));          // in : NewFilletEdge's filletCorner, gets stored in rpNewFilletEdge->m_cpCorner
                                                               //      NULL to ignore, default:[NULL]
        }
      else 
        {
          SER( SmFilletEdge::MakeFilletEdge( m_pPseudoBrep,    // in : target Brep to receive new topology objects
                                             pOtherVert,       // in : start of new FilletEdge
                                             pVertOnSideFace,  // in : end   of new FilletEdge
                                             pFilletEdge2,     // out: newly allocated FilletEdge - no Curve or UVTrimCurve data
                                             this ));          // in : NewFilletEdge's filletCorner, gets stored in rpNewFilletEdge->m_cpCorner
                                                               //      NULL to ignore, default:[NULL]
        }
      pFilletEdge2->SetFilletEdgeType( SM_FE_FILLET_X_FILLET );

      pPrimEU = (SmFilletEdgeuse*)pFilletEdge2->GetPrimaryEdgeuse();
      SmFilletEdgeuse * pMateEU = (SmFilletEdgeuse*)pPrimEU->GetMate();

      pPrimEU->SetFilletGeom( pLargeRadGeom );
      pMateEU->SetFilletGeom( pSmallRadGeom );

      lIndex--;
      SmFilletEdge * pRemovedEdge = m_vEdges[lIndex];
      m_vEdges.RemoveAt( lIndex );
      SM_ASSERT( pRemovedEdge != NULL );
      if(pRemovedEdge) { delete pRemovedEdge ; pRemovedEdge = NULL; }

      if ( bInsertAfter )
        {
          m_vEdges.Add( pFilletEdge1 );
          m_vEdges.Add( pFilletEdge2 );
        }
      else
        {
          m_vEdges.Add( pFilletEdge2 );
          m_vEdges.Add( pFilletEdge1 );
        }
      sClean1.Clear();
      ii++;

    } // end iter ii, every m_vVertices vertex

  // all done
  return SM_SUCCESS;

} // end SmFilletNx2Corner::AdjustTopology

/*******************************************************************//**
PURPOSE: Calculate geometry of a Nx2 concave corner patch where two
    adjacent rails may not meet at a single vertex (with type: SM_FV_RAIL_X_EDGEUSE).
    In that case, two patches shall be created:
    (1) a small patch to fill in the tiny gap &
    (2) a regular 3-sided COONS patch to blend two fillets.

NOTES: This corner should have either 3 or 6 boundary edges
***********************************************************************/
SmStatus SmFilletNx2Corner::CreateCornerPatch()
{
  ULONG ii, lTotalEdges = m_vEdges.GetSize();

  SmFilletEdge * apData1[3];
  SmFilletEdge * apData2[3];
  SmFilletVertex * apData3[2];
  SmTArray<SmFilletEdge*> sFirstThreeEdges(3,apData1);
  SmTArray<SmFilletEdge*> sLastThreeEdges(3,apData2);
  SmTArray<SmFilletVertex*> sLastTwoVerts(2,apData3);
  SmSurface * pEndPatch =  NULL;
  if (lTotalEdges > 3) 
    {
      for (ii=0; ii<3; ii++) 
        { sFirstThreeEdges.Add(m_vEdges[ii]) ; }

      for (ii=3; ii<m_vEdges.GetSize(); ii++) 
        { sLastThreeEdges.Add(m_vEdges[ii]) ; }

      if (lTotalEdges == 6) 
        {
          // Use the last 3 edges to create a small patch
          m_vEdges.ReSet();
          m_vEdges.Append(sLastThreeEdges);
          // Create a small corner face
          SER(SmFilletCorner::CreateEndPatch());

          // Temporarily remove the small patch
          pEndPatch = m_vSurfaces[0];
          m_vSurfaces.ReSet();
        }

      m_vEdges.ReSet();
      m_vEdges.Append(sFirstThreeEdges);

      // Temporarily remove the last two vertices
      for(ii=m_vVertices.GetSize()-1; ii>=3; ii--) 
        {
          sLastTwoVerts.Add(m_vVertices[ii]) ;
          m_vVertices.RemoveLast() ;
        }
    }

  // Create a 3-sided corner patch
  SER(SmFilletCorner::CreateCornerPatch());

  // Put the end patch back
  if (pEndPatch) 
    { m_vSurfaces.Add(pEndPatch) ; }

  if (sLastThreeEdges.GetSize() > 0) 
    {
      m_vEdges.Append(sLastThreeEdges) ;
      m_vVertices.Append(sLastTwoVerts) ;
    }

  return SM_SUCCESS;

} // end SmFilletNx2Corner::CreateCornerPatch

/*******************************************************************//**
PURPOSE: A 4x3 corner represents a corner with 4 adjacent
    edges and three of them are filleted. Typically, a blending corner
    patch will be generated.

NOTES:
***********************************************************************/
SmFillet4x3Corner::SmFillet4x3Corner
  (const SmContext      & crContext,
   const SmVertex       * cpVertex,
   SmTArray<SmEdgeuse*> * pSolverEUs,
   SmTArray<SmEdgeuse*> * pAllEUs,
   SmFilletExecutive    * pExec,
   double                 dApproxTol3d,
   double                 dTangencyTolRadians)
 : SmFilletCorner(crContext,cpVertex,pSolverEUs,pAllEUs,pExec,
                  dApproxTol3d,dTangencyTolRadians)
{

} // end SmFillet4x3Corner::SmFillet4x3Corner constructor

/*******************************************************************//**
PURPOSE: Make corner topology which consists of an array of
    SmFilletVertex and an array of SmFilletEdge.

NOTES:
***********************************************************************/
SmStatus SmFillet4x3Corner::MakeCornerTopology
  ()
{
int iDebugLevel = 0;
#ifdef SM_DEBUG_CODE
iDebugLevel = DebugLevel();
#endif // SM_DEBUG_CODE

    ULONG lTotalEUs = m_pAllEUs->GetSize();
    ULONG lTotalSolverEUs = m_pSolverEUs->GetSize();
    SM_ASSERT(lTotalEUs == lTotalSolverEUs+1);
    ULONG ii;

    // Allocate three Fillet Vertices
    for (ii=0; ii<lTotalSolverEUs; ii++) {
        SmFilletVertex * pFV = new (m_pPseudoBrep) SmFilletVertex(this);
        m_vVertices.Add(pFV);
    }

    SmFilletExecutive * pExec = GetFilletExecutive();

    SmEdgeuse * pCurrEU = (*m_pAllEUs)[0];
    SmFilletSolver * pCurrFilSolver = pExec->GetFilletSolverOfEdgeuse(pCurrEU);
    ULONG lCurrIndex = 0;
    SmFilletVertex * pCurrFV = m_vVertices[lCurrIndex];
    for (ii=0; ii<lTotalEUs; ii++) {
        SmEdgeuse * pNextEU = m_pAllEUs->GetAt((ii+1)%lTotalEUs);
        SmFilletSolver * pNextFilSolver = pExec->GetFilletSolverOfEdgeuse(pNextEU);

        if (pNextFilSolver == NULL) {
            pCurrFilSolver = NULL;
            pCurrEU = pNextEU;
            continue;
        }
        SmFilletGeom * pNextFilletGeom = pNextFilSolver->GetFirstFilletGeom();

        if (pCurrFilSolver == NULL) {
            SmEdgeuse * pPrevEU = m_pAllEUs->GetAt(((ii+lTotalEUs)-1)%lTotalEUs);
            SmFilletSolver * pPrevFilSolver = pExec->GetFilletSolverOfEdgeuse(pPrevEU);
            NER(pPrevFilSolver);
            NER(pNextFilSolver);
            SmFilletGeom * pPrevFilletGeom = pPrevFilSolver->GetFirstFilletGeom();
            SmEdgeuse * pSideEU = sm_FindSideEdgeuse(pPrevEU,pCurrEU, iDebugLevel );
            pCurrFV->SetPointClass(SM_PC_EDGEUSE, pSideEU);
            pCurrFV->SetFilletVertexType(SM_FV_RAIL_X_EDGEUSE);

            // Create rail edgeuses
            SmFilletEdgeuse * pNewEdgeuse = NULL;
            ULONG lRailIndex = pPrevFilSolver->FindIndexOfRailXSideEdgeuse(pSideEU);
            ULONG lOtherRailIndex = pNextFilSolver->FindIndexOfRailXSideEdgeuse(
                pSideEU->GetRadial());
            SER(pPrevFilletGeom->MakeRailEdgeuse(m_pPseudoBrep,
                lRailIndex,pCurrFV,pNewEdgeuse));
            SER(pNextFilletGeom->MakeRailEdgeuse(m_pPseudoBrep,
                lOtherRailIndex,pCurrFV,pNewEdgeuse));
        }
        else {
            SmFace * pCommonFace = sm_GetFaceByTwoEdgeuses(pCurrEU,pNextEU,NULL);
            NER(pCommonFace);//Unrecoverable error
            pCurrFV->SetPointClass(SM_PC_FACE,pCommonFace);
            pCurrFV->SetFilletVertexType(SM_FV_RAIL_X_RAIL);

            // Create rail edgeuses
            SmFilletGeom * pCurrFilletGeom = pCurrFilSolver->GetFirstFilletGeom();
            SmFilletEdgeuse * pNewEdgeuse = NULL;
            ULONG lRailIndex = pCurrFilSolver->FindIndexOfRailOnFace(pCommonFace);
            ULONG lOtherRailIndex = pNextFilSolver->FindIndexOfRailOnFace(pCommonFace);
            SER(pCurrFilletGeom->MakeRailEdgeuse(m_pPseudoBrep,
                lRailIndex,pCurrFV,pNewEdgeuse));
            SER(pNextFilletGeom->MakeRailEdgeuse(m_pPseudoBrep,
                lOtherRailIndex,pCurrFV,pNewEdgeuse));
        }

        // Make fillet edge
        lCurrIndex++;
        SmFilletVertex * pNextFV = m_vVertices[lCurrIndex%lTotalSolverEUs];
        SmFilletEdge * pFilletEdge = NULL;
        SER(SmFilletEdge::MakeFilletEdge(m_pPseudoBrep,  // in : target Brep to receive new topology objects
                                         pCurrFV,        // in : start of new FilletEdge
                                         pNextFV,        // in : end   of new FilletEdge
                                         pFilletEdge,    // out: newly allocated FilletEdge - no Curve or UVTrimCurve data
                                         this));         // in : NewFilletEdge's filletCorner, gets stored in rpNewFilletEdge->m_cpCorner
                                                         //      NULL to ignore, default:[NULL]
        pFilletEdge->SetFilletEdgeType(SM_FE_CROSS_SECTION);
        SmFilletEdgeuse * pPrimEU = (SmFilletEdgeuse*)pFilletEdge->
            GetPrimaryEdgeuse();
        pPrimEU->SetFilletGeom(pNextFilletGeom);
        m_vEdges.Add(pFilletEdge);

        pCurrFilSolver = pNextFilSolver;
        pCurrEU = pNextEU;
        pCurrFV = pNextFV;

    } // end for all EUs

//    m_bBlending = TRUE;

    return SM_SUCCESS;

} // end SmFillet4x3Corner::MakeCornerTopology

/*******************************************************************//**
PURPOSE: Create an open-end corner (i.e. corner with lamina edge) to
    determine the trimming boundary of fillet at this corner.
    Typically, this will be needed when we do shell-filleting.

NOTES: For now, it will only handle 3x1 case.
***********************************************************************/
SmFilletOpenCorner::SmFilletOpenCorner
  (const SmContext      & crContext,
   const SmVertex       * cpVertex,
   SmTArray<SmEdgeuse*> * pSolverEUs,
   SmTArray<SmEdgeuse*> * pAllEUs,
   SmFilletExecutive    * pExec,
   double                 dApproxTol3d,
   double                 dTangencyTolRadians)
 : SmFilletNx1Corner(crContext,cpVertex,pSolverEUs,pAllEUs,pExec,
                     dApproxTol3d,dTangencyTolRadians)
{

} // end SmFilletOpenCorner::SmFilletOpenCorner constructor

/*******************************************************************//**
PURPOSE: Make corner topology which consists of an array of
    SmFilletVertex and an array of SmFilletEdge. Since we are dealing with
    OPEN end, the corner actually consists of only one fillet edge.

NOTES: For now, it will only handle 3x1 case.
***********************************************************************/
SmStatus SmFilletOpenCorner::MakeCornerTopology
  ()
{
    SmTArray<SmEdge*> sEdges;
    m_cpVertex->GetEdges(sEdges);
    SmEdgeuse * pFilEdgeuse = (*m_pSolverEUs)[0];

    SmEdgeuse * pEU[2]; // Adjacent edgeuses of filleted edgeuse
    pEU[0] = pFilEdgeuse->GetCWEdgeuse();
    pEU[1] = pFilEdgeuse->GetRadial()->GetCCWEdgeuse();
#ifdef SM_DEBUG_CODE
    if ( DebugLevel() > 0 ) {
        smgfx_SetLook(1,2, 1,0,0); pFilEdgeuse->Draw(); sm_GraphicsLoop();
        smgfx_SetLook(1,2, 1,0,0); pFilEdgeuse->GetRadial()->Draw(); sm_GraphicsLoop();
        smgfx_SetLook(1,2, 0,0,1); pEU[0]->Draw(); sm_GraphicsLoop();
        smgfx_SetLook(1,2, 0,0,1); pEU[1]->Draw(); sm_GraphicsLoop();
        sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

    SmFilletSolver * pFilSolver = m_pExecutive->GetFilletSolverOfEdgeuse(pFilEdgeuse);
    NER(pFilSolver);
    SmFilletGeom * pFilletGeom = pFilSolver->GetFirstFilletGeom();
    ULONG ii;
    for (ii=0; ii<2; ii++) {
        SmFilletVertex * pFV = new (m_pPseudoBrep) SmFilletVertex(this);
        pFV->SetPointClass(SM_PC_EDGEUSE, pEU[ii]);
        pFV->SetFilletVertexType(SM_FV_RAIL_X_EDGEUSE);
        m_vVertices.Add(pFV);
        ULONG lRailIndex = pFilSolver->FindIndexOfRailXSideEdgeuse(pEU[ii]);
        // Create rail-edgeuse using this vertex
        SmFilletEdgeuse * pNewEdgeuse = NULL;
        SER(pFilletGeom->MakeRailEdgeuse(m_pPseudoBrep,
            lRailIndex,pFV,pNewEdgeuse));
    }
    SmFilletEdge * pFilletEdge = NULL;
    SER(SmFilletEdge::MakeFilletEdge(m_pPseudoBrep,  // in : target Brep to receive new topology objects
                                     m_vVertices[0], // in : start of new FilletEdge
                                     m_vVertices[1], // in : end   of new FilletEdge
                                     pFilletEdge,    // out: newly allocated FilletEdge - no Curve or UVTrimCurve data
                                     this));         // in : NewFilletEdge's filletCorner, gets stored in rpNewFilletEdge->m_cpCorner
                                                     //      NULL to ignore, default:[NULL]
    pFilletEdge->SetFilletEdgeType(SM_FE_FILLET_END);
    SmFilletEdgeuse * pPrimEU = (SmFilletEdgeuse*)pFilletEdge->GetPrimaryEdgeuse();
    pPrimEU->SetFilletGeom(pFilletGeom);
    m_vEdges.Add(pFilletEdge);

    return SM_SUCCESS;

} // end SmFilletOpenCorner::MakeCornerTopology

/*******************************************************************//**
PURPOSE: Create an open-end corner (i.e. corner with lamina edge) to
    determine the trimming boundary of fillet at this corner.
    Typically, this will be needed when we do shell-filleting.

NOTES: For now, it will only handle 3x1 case.
***********************************************************************/
SmFilletDegenerateCorner::SmFilletDegenerateCorner
  (const SmContext      & crContext,
   const SmVertex       * cpVertex,
   SmTArray<SmEdgeuse*> * pSolverEUs,
   SmTArray<SmEdgeuse*> * pAllEUs,
   SmFilletExecutive    * pExec,
   double                 dApproxTol3d,
   double                 dTangencyTolRadians)
 : SmFilletCorner(crContext,cpVertex,pSolverEUs,pAllEUs,pExec,
                  dApproxTol3d,dTangencyTolRadians)
{
    m_bDegenerate = TRUE;

} // end SmFilletDegenerateCorner::SmFilletDegenerateCorner constructor

/*******************************************************************//**
PURPOSE: Make corner topology which consists of an array of
    SmFilletVertex and an array of SmFilletEdge. Since we are dealing with
    OPEN end, the corner actually consists of only one fillet edge.

NOTES: For now, it will only handle 3x1 case.
***********************************************************************/
SmStatus SmFilletDegenerateCorner::MakeCornerTopology
  ()
{
    SmTArray<SmEdge*> sEdges;
    m_cpVertex->GetEdges(sEdges);
    SmEdgeuse * pFilEdgeuse = (*m_pSolverEUs)[0];

    SmEdgeuse * pEU[2]; // Adjacent edgeuses of filleted edgeuse
    pEU[0] = pFilEdgeuse->GetCWEdgeuse();
    pEU[1] = pFilEdgeuse->GetRadial()->GetCCWEdgeuse();

    SmFilletSolver * pFilSolver = m_pExecutive->GetFilletSolverOfEdgeuse(pFilEdgeuse);
    SmFilletVertex * pFV = new (m_pPseudoBrep) SmFilletVertex(this);
    pFV->SetPointClass(SM_PC_VERTEX, (SmObject*)m_cpVertex);
    pFV->SetFilletVertexType(SM_FV_ON_VERTEX);
    SmPoint3d sCornerPnt = m_cpVertex->GetPoint();
    pFV->SetPoint(sCornerPnt);
    pFV->SetStatus(SM_FIL_PROCESSED);
    m_vVertices.Add(pFV);

#ifdef SM_DEBUG_CODE
    if ( DebugLevel() > 0 ) {
        smgfx_SetLook(1,2, 1,0,0);
        if (FALSE) {
            pFilEdgeuse->Draw(); sm_GraphicsLoop();
            pFilEdgeuse->GetRadial()->Draw(); sm_GraphicsLoop();
            sm_GraphicsLoop();
        }
        smgfx_SetLook(1,2, 0,1,0);
        if (FALSE) {
            pEU[0]->Draw(); sm_GraphicsLoop();
            pEU[1]->Draw(); sm_GraphicsLoop();
            sm_GraphicsLoop();
        }
        smgfx_SetLook(4,10, 0,1,1); pFV->Draw(); sm_GraphicsLoop();
        sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE
    // Drop sCornerPnt onto 2 surfaces
    SmPoint2d sUV[2];
    ULONG ii;
    for (ii=0; ii<2; ii++) {
        const SmSurface * pBase = pFilSolver->GetSurface(ii)->GetBaseSurface();
        SmSolution sSData[16];
        SmSolutionArray sSolutions(16,sSData);
#ifdef SM_DEBUG_CODE
        if ( DebugLevel() > 0 ) {
            smgfx_SetLook(1,2, 0,0,0); pBase->DrawUV(5,5); sm_GraphicsLoop();
            sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE
        SmSurfaceCache *pSC = smsurf_GetSurfaceCache(pBase); NER(pSC);
        SmCacheCheckOutIn sCheckIO(pSC);
        {
        // Turn off point testing so GlobalPointSolve() will keep all point solutions
        //   without classifying the solution point against the trim boundaries.
        SmTemporaryChangeValue<SmBoolean> sChange(pSC->m_bPointTestEnabled,FALSE);

        SER(pBase->GlobalPointSolve(pBase->GetNaturalUVDomain(),SM_SO_INTERSECT,
            sCornerPnt,m_cpVertex->GetTolerance()*10.0,NULL,SM_SR_SINGLE,sSolutions));
        }
        if (sSolutions.GetSize() != 1) SER(SM_ERR);
        sUV[ii].x = sSolutions[0].m_vStart[0];
        sUV[ii].y = sSolutions[0].m_vStart[1];
    }

    SmFilletGeom * pFilletGeom = pFilSolver->GetFirstFilletGeom();
    for (ii=0; ii<2; ii++) {
        ULONG lRailIndex = pFilSolver->FindIndexOfRailXSideEdgeuse(pEU[ii]);
        // Create rail-edgeuse using this vertex
        SmFilletEdgeuse * pNewEdgeuse = NULL;
        SER(pFilletGeom->MakeRailEdgeuse(m_pPseudoBrep,
            lRailIndex,pFV,pNewEdgeuse));
        SmFilletVertexuse * pVU = (SmFilletVertexuse*)pNewEdgeuse->GetVertexuse();
        SmTsectPnt & rTsectPnt = pVU->GetTsectPnt();
        rTsectPnt.UVPos(   lRailIndex ) = sUV[   lRailIndex ];
        rTsectPnt.UVPos( 1-lRailIndex ) = sUV[ 1-lRailIndex ];
        rTsectPnt.m_ePointType = SM_IP_TANGENT_POINT;
    }

    return SM_SUCCESS;

} // end SmFilletDegenerateCorner::MakeCornerTopology

#ifdef SM_DEBUG_CODE
/****************************************************************
PURPOSE: Debug level.
****************************************************************/
int SmFilletCorner::DebugLevel()
  { return m_pExecutive->DebugLevel(); }
#endif // SM_DEBUG_CODE



