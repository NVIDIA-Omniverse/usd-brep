// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmGraphicsNLib.cpp
* PURPOSE: Draw NLib objects by upgrading to Sm first
*          Primarily used by Examples.
**********************************************************************/

#include <StdAfx.h>
#include <SmGraphicsNLib.h>
#include <nurbs.h>
#include <SmGraphicsExtern.h>
#include <SmBSplineCurve.h>
#include <SmBSplineSurface.h>

// Static context only for the functions below
static SmContext *s_pContext = NULL;


/**************************************************************
PURPOSE ---

USAGE NOTES ---
**************************************************************/
SM_EXPORT void smgfx_DrawNPt(NL_POINT *pnt)
{
  SmPoint3d Pt;

  if (pnt)
    {
      Pt.x = pnt->x;
      Pt.y = pnt->y;   
      Pt.z = pnt->z;
      Pt.Draw(); sm_GraphicsLoop();
    }
  return;

} // end smgfx_DrawNPnt


/**************************************************************
PURPOSE ---

USAGE NOTES ---
**************************************************************/
SM_EXPORT void smgfx_DrawNPts( int n, NL_POINT *pnts )
{
  // draw NLib 1d array of points
  SmPoint3d Pt;

  smgfx_SetLook( 2,4, 1,0,0); 
  for(int i = 0; i <= n; i++)
  {
    NL_POINT *P = &pnts[i];
    Pt.x = P->x;
    Pt.y = P->y;
    Pt.z = P->z;
    Pt.Draw(); sm_GraphicsLoop();
  }
  smgfx_SetLook( 2,3, 0,0,0);

  return;

} // end smgfx_DrawNPts

/**************************************************************
PURPOSE ---

USAGE NOTES ---
**************************************************************/
SM_EXPORT void smgfx_DrawNCrv( NL_CURVE *crv )
{
  if(crv && crv->pol == NULL)
    return;

  SmBSplineCurve sCurve( crv, N_CrvIs3d( crv ) ? 3 : 2, TRUE );
  sCurve.Draw(); sm_GraphicsLoop();
  sm_GraphicsLoop();

  return;

} // end smgfx_DrawNCrv

/*******************************************************************//**
PURPOSE: Draw CrossTangent values      

NOTES: A specialized draw routine for checking the input
  to the N_FillNSidedHole() call.

  pTangCur is a gw_Curve showe values are crossTangent vectors with
  associated base points found by evaluating pBaseCur.  The crossTangent
  curve is used to define the G1 constraint specified to the N_FillNSidedHole()
  function.
***********************************************************************/
void smgfx_DrawCrossTangentNCrv
  (
  NL_CURVE *pCrossTangCur,     // in : Curve which represents Cross Tangent values
  NL_CURVE *pBaseCur           // in : Curve which represents Cross Tangent base points
  )
{ 
  SmBSplineCurve sCrossTangentCurve(pCrossTangCur, N_CrvIs3d(pCrossTangCur) ? 3 : 2, TRUE) ;
  SmBSplineCurve sBaseCurve(pBaseCur, N_CrvIs3d(pBaseCur) ? 3 : 2, TRUE) ;
   
#ifdef SM_GFX_CODE

    smgfx_Open();
    SmExtent1d sIvl = sCrossTangentCurve.GetNaturalInterval() ;
    for (double dT=0.0; dT<=1.0; dT=dT+0.1) 
      {
        SmPoint3d sStartPnt;
        sBaseCurve.EvaluatePoint(sIvl.Evaluate(dT), sStartPnt);
        SmVector3d sTangent;
        sCrossTangentCurve.EvaluatePoint(sIvl.Evaluate(dT), sTangent);
        sTangent.Draw(&sStartPnt);
      }
  smgfx_Close() ;

#endif // end SM_GFX_CODE

} // end smgfx_DrawCrossTangentNCrv

/**************************************************************
PURPOSE ---

USAGE NOTES ---
**************************************************************/
SM_EXPORT void smgfx_DrawNCrvs(int ncrv, NL_CURVE **crvs)
{ 
    for (int i = 0; i < ncrv; i++)
    {    
        SM_ASSERT(crvs[i]->pol != NULL);
        SmBSplineCurve sCurve( crvs[i], N_CrvIs3d( crvs[i] ) ? 3 : 2, TRUE );
        sCurve.Draw(); sm_GraphicsLoop();
        sm_GraphicsLoop();  
    }
    return;

} // end smgfx_DrawNCrvs



/**************************************************************
PURPOSE ---

USAGE NOTES --- FALSE = don't run AssertValidAndHeal on New BSplineSurface
**************************************************************/
SM_EXPORT void smgfx_DrawNSrf( NL_SURFACE *pSur )
{
  if(pSur && pSur->net == NULL)
    return;

  SmBSplineSurface sSurface(pSur, TRUE, NULL) ; 
  sSurface.DrawUV(); sm_GraphicsLoop();
  sm_GraphicsLoop();
  return;

} // end smgfx_DrawNSrf


/**************************************************************
PURPOSE ---

USAGE NOTES ---
**************************************************************/
SM_EXPORT void smgfx_DrawNTri(NL_POINT *pnt1, NL_POINT *pnt2, NL_POINT *pnt3)
{
    SmPoint3d Pt1, Pt2, Pt3;

    if (pnt1)
    {
        Pt1.x = pnt1->x;
        Pt1.y = pnt1->y;   
        Pt1.z = pnt1->z;

        Pt2.x = pnt2->x;
        Pt2.y = pnt2->y;   
        Pt2.z = pnt2->z;

        Pt3.x = pnt3->x;
        Pt3.y = pnt3->y;   
        Pt3.z = pnt3->z;          
        
        smgfx_DrawTri(&Pt1, &Pt2, &Pt3);
    }
    return;

} // end smgfx_DrawNTri

/**************************************************************
PURPOSE --- Draw 2 points as a line

USAGE NOTES ---
**************************************************************/
SM_EXPORT void smgfx_DrawLine( SmPoint3d *P0, SmPoint3d *P1 )
{
  if(s_pContext == NULL)
      s_pContext = new SmContext();
  
  SmBSplineCurve *pLine = NULL;
  SmBSplineCurve::CreateLineSegment( *s_pContext, 3, *P0, *P1, pLine );
  pLine->Draw(); sm_GraphicsLoop();

} // end smgfx_DrawLine

/**************************************************************
PURPOSE --- Draw 3 points as a triangle

USAGE NOTES ---
**************************************************************/
SM_EXPORT void smgfx_DrawTri( SmPoint3d *P0, SmPoint3d *P1, SmPoint3d *P2 )
{
  if(s_pContext == NULL)
    s_pContext = new SmContext();

  SmBSplineCurve *pLine = NULL;
  SmBSplineCurve::CreateLineSegment( *s_pContext, 3, *P0, *P1, pLine );
  pLine->Draw(); sm_GraphicsLoop();

  SmBSplineCurve::CreateLineSegment( *s_pContext, 3, *P1, *P2, pLine );
  pLine->Draw(); sm_GraphicsLoop();

  SmBSplineCurve::CreateLineSegment( *s_pContext, 3, *P2, *P0, pLine );
  pLine->Draw(); sm_GraphicsLoop();

  return;

} // end smgfx_DrawTri


// These are potential functions that we may want to use from an application
#if 0
/**************************************************************
PURPOSE --- Add a copy of a PolyBrep to the UI UserPolyBreps array

USAGE NOTES --- inputs have to be copied to preserve context and scope
**************************************************************/
void UserTest::Draw(SmPolyBrep *pPolyBrep) 
{
  if (pPolyBrep)    
    {
      // copy the PolyBrep saving a map of copy obj ptrs to orig obj ptrs
      //  SmCopyPolyBrepMap *pCopyPolyBrepMap = new (*pPolyBrep->GetContext()) SmCopyPolyBrepMap() ; 
      SmCopyPolyBrepMap *pCopyPolyBrepMap = new (*s_pContext) SmCopyPolyBrepMap() ;
      SmPolyBrep        *pPolyBrepCopy    = new (*s_pContext) SmPolyBrep(*pPolyBrep, pCopyPolyBrepMap);

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
      if(bDebugMe)
        {
          if(pPolyBrep)        pPolyBrep->Dump() ;
          if(pPolyBrepCopy)    pPolyBrepCopy->Dump() ;
          if(pCopyPolyBrepMap) pCopyPolyBrepMap->Dump() ;
        }
#endif // SM_DEBUG_CODE

      // add PolyBrepCopy, PolyBrepCopyMap, and a CopyPolyBrep->CopyPolyBrepMap map entry to user interface
      pCopyPolyBrepMap->SetPolyBreps(pPolyBrep, pPolyBrepCopy) ;
      UserPolyBreps.Add(pPolyBrepCopy) ;
      UserPolyBrepMaps.Add(pCopyPolyBrepMap) ;
      UserPolyBrepToMap.SetAt(pPolyBrepCopy, pCopyPolyBrepMap) ;
    }

  // without PolyBrep object mapping
  //      if (pPolyBrep) 
  //        {
  //          SmPolyBrep *pPolyBrepCopy = new(*s_pContext) SmPolyBrep(*pPolyBrep);
  //          UserPolyBreps.Add(pPolyBrepCopy);
  //        }

} // end UserTest::Draw PolyBrep

/**************************************************************
PURPOSE --- Add a copy of a Brep to the UI UserBreps array

USAGE NOTES ---
**************************************************************/
void UserTest::Draw(SmBrep *pBrep) 
{
  if (pBrep) 
    {
      // copy the Brep saving a map of copy obj ptrs to orig obj ptrs
      SmCopyBrepMap *pCopyBrepMap = new (*s_pContext) SmCopyBrepMap() ;
      SmBrep *pBrepCopy = new(*s_pContext) SmBrep(*pBrep, pCopyBrepMap);

      // add BrepCopy, BrepCopyMap, and a CopyBrep->CopyBrepMap map entry to user interface
      pCopyBrepMap->SetBreps(pBrep, pBrepCopy) ;
      UserBreps.Add(pBrepCopy) ;
      UserBrepMaps.Add(pCopyBrepMap) ;
      UserBrepToMap.SetAt(pBrepCopy, pCopyBrepMap) ;
    }

} // end UserTest::Draw Brep

/**************************************************************
PURPOSE --- Add a copy of each Shell->Face to the UI UserFaces array

USAGE NOTES ---
**************************************************************/
void UserTest::Draw(SmShell *pShell)

// note each face is copied into a new brep so that
// all of the exterior face edges will appear lamina
// and colored blue, even though original shell may have
// manifold edges
{
  if ( pShell) 
    {
      SmTArray <SmFaceuse*> sFaceuses;
      pShell->GetFaceuses( sFaceuses );
      ULONG nfus = sFaceuses.GetSize();
      if (nfus > 0 ) 
        {
          for (ULONG i=0; i<nfus; i++)
            {
              SmFace *pFace = sFaceuses[i]->GetFace();
              UserTest::Draw(pFace);  // draws each face in its own brep.
            }
        }
    }
} // end UserTest::Draw Shell



/**************************************************************
PURPOSE ---

USAGE NOTES ---
**************************************************************/
void UserTest::Draw(SmBSplineSurface *pSrf) 
{
    if (pSrf) 
    {
        // add copy of surface to Draw array
        SmBSplineSurface *pSrfCopy =  new(*s_pContext) SmBSplineSurface(*pSrf);
        UserSurfaces.Add(pSrfCopy);

        // map original to copy surfaces
        UserSurfaceMap.SetAt(pSrf, pSrfCopy) ;
    }

} // end UserTest::Draw(SmBSplineSurface)

/**************************************************************
PURPOSE ---

USAGE NOTES ---
**************************************************************/
void UserTest::Draw(SmSurface *pSrf) 
{  
    if (pSrf)
      {
        SmSurface *pSrfCopy = NULL ;
        pSrf->Copy(*s_pContext, pSrfCopy) ;
        UserSurfaces.Add(pSrfCopy);

        // map original to copy surfaces
        UserSurfaceMap.SetAt(pSrf, pSrfCopy) ;
      }

} // end UserTest::Draw(SmSurface)

/**************************************************************
PURPOSE ---

USAGE NOTES ---
**************************************************************/
void UserTest::Draw(SmVolume *pVolume) 
{  
    if (pVolume)
      {
        SmVolume *pVolumeCopy = NULL ;
        pVolume->Copy(*s_pContext, pVolumeCopy) ;
        UserVolumes.Add(pVolumeCopy);

        // map original to copy volumes
        UserVolumeMap.SetAt(pVolume, pVolumeCopy) ;
      }

} // end UserTest::Draw(SmVolume)

/**************************************************************
PURPOSE ---

USAGE NOTES ---
**************************************************************/
void UserTest::Draw(SmFace *pFace) 
{
  if (pFace) 
    {        
      SmBrep *pBrep = pFace->GetBrep();
      SmBrep *pBrepFace = new(*s_pContext) SmBrep();
      SmTArray < SmFace *> sFaces;
      sFaces.Add(pFace);
      pBrep->CopyFaces(sFaces, pBrepFace);
      UserBreps.Add(pBrepFace);
    }

} // end UserTest::Draw(SmFace)

/**************************************************************
PURPOSE ---

USAGE NOTES ---
**************************************************************/
void UserTest::Draw(SmEdge *pEdge) 
{
  if (pEdge) 
    {        
      SmBSplineCurve *pCrv  =(SmBSplineCurve *)pEdge->GetCurve();
      SmExtent1d dInterval  = pEdge->GetInterval();
      SmBSplineCurve *pCrvT =  new(*s_pContext) SmBSplineCurve(*pCrv);
      pCrvT->Trim(dInterval);
      UserCurves.Add(pCrvT);
    }

} // end UserTest::Draw(SmEdge)

/**************************************************************
PURPOSE ---

USAGE NOTES ---
**************************************************************/
void UserTest::Draw(SmVertex *pVertex) 
{
  if (pVertex)
    {       
      SmPoint3d Pnt = pVertex->GetPoint();
      Draw(Pnt, pVertex);
    }

} // end UserTest::Draw(SmVertex)

/**************************************************************
PURPOSE ---

USAGE NOTES ---
**************************************************************/
void UserTest::Draw(SmPolyFace *pPolyFace) 
{
  if (pPolyFace) 
    {        
      SmPolyBrep *pPolyBrep = pPolyFace->GetPolyBrep();
      SmPolyBrep *pPolyBrepFace = new(*s_pContext) SmPolyBrep(pPolyBrep->GetTolerance());
      SmTArray < SmPolyFace *> sPolyFaces;
      sPolyFaces.Add(pPolyFace);
      pPolyBrep->CopyFaces(sPolyFaces, pPolyBrepFace, SM_MT_NOMARK);
      UserPolyBreps.Add(pPolyBrepFace);
    }

} // end UserTest::Draw(SmPolyFace)

/**************************************************************
PURPOSE ---

USAGE NOTES ---
**************************************************************/
void UserTest::Draw(SmPolyEdge *pPolyEdge) 
{
  if (pPolyEdge) 
    { 
      SmPoint3d sPoint0 = pPolyEdge->GetStartPoint() ;
      SmPoint3d sPoint1 = pPolyEdge->GetEndPoint() ;
      Draw(&sPoint0, &sPoint1, pPolyEdge) ;
    }

} // end UserTest::Draw(SmPolyEdge)

/**************************************************************
PURPOSE ---

USAGE NOTES ---
**************************************************************/
void UserTest::Draw(SmPolyVertex *pPolyVertex) 
{
  if (pPolyVertex)
    {       
      SmPoint3d Pnt = pPolyVertex->GetPoint();
      Draw(Pnt, pPolyVertex);
    }

} // end UserTest::Draw(SmPolyVertex)

/**************************************************************
PURPOSE ---

USAGE NOTES ---
**************************************************************/
void UserTest::Draw(SmPoint3d Point, SmObject *pOptObject) 
{
  UserPoints.Add(SmUserPoint(Point, pOptObject));

} // end UserTest::Draw(SmPoint3d)

/**************************************************************
PURPOSE ---

USAGE NOTES ---
**************************************************************/
void UserTest::Draw(SmCurve *pCrv) 
{  
    if (pCrv)
      {
        SmCurve *pCrvCopy = NULL ;
        pCrv->Copy(*s_pContext, pCrvCopy) ;
        UserCurves.Add(pCrvCopy);

        // map original to copy curves
        UserCurveMap.SetAt(pCrv, pCrvCopy) ;
      }

} // end UserTest::Draw(SmCurve)

/**************************************************************
PURPOSE ---

USAGE NOTES ---
**************************************************************/
void UserTest::Draw(SmBSplineCurve *pCrv) 
{
    if (pCrv)
    {
        SmCurve *pCrvCopy = NULL;
        pCrv->Copy(*s_pContext, pCrvCopy);
        UserCurves.Add(pCrvCopy);

        // map original to copy curves
        UserCurveMap.SetAt(pCrv, pCrvCopy) ;
    }

} // end UserTest::Draw(SmBSplineCurve)

/**************************************************************
PURPOSE ---

USAGE NOTES ---
**************************************************************/
void UserTest::Draw(SmCompositeCurve *pCrv)
{
    if (pCrv)
    {
        for (ULONG i = 0; i < pCrv->GetNumSegments(); i++)
        {
            SmBSplineCurve* pC =(SmBSplineCurve *)(pCrv->GetCurveSegment(i)->m_pParentCurve);
            SmBSplineCurve *pCrvCopy =  new(*s_pContext) SmBSplineCurve(*pC);
            UserCurves.Add(pCrvCopy);
        }
    }

} // end UserTest::Draw(SmCompositeCurve)


/**************************************************************
PURPOSE ---

USAGE NOTES ---
**************************************************************/
void UserTest::Draw(SmTArray <SmPoint3d> &rPoints) 
{
  for (ULONG i = 0; i < rPoints.GetSize(); i++) 
    {
      UserPoints.Add(SmUserPoint(rPoints[i], &rPoints));
    }

} // end UserTest::Draw(SmTArray <SmPoint3d>)

/**************************************************************
PURPOSE --- Draws Open PolyLine as a sequence of Lines

USAGE NOTES --- To draw a closed PolyLine, 
  make sure the 1st and last points in the input array
  are at the same location.
**************************************************************/
void UserTest::DrawPolyLine(SmTArray <SmPoint3d> &rPoints, SmObject *pOptObject) 
{
  ULONG i0, i1 ;

  for (i0=0,i1=1;i1< rPoints.GetSize();i0++,i1++) 
    {
      UserLines.Add(SmUserLine(rPoints[i0], rPoints[i1], pOptObject));
    }

} // end UserTest::Draw(SmTArray <SmPoint3d>)

/**************************************************************
PURPOSE ---

USAGE NOTES ---
**************************************************************/
void UserTest::Draw(SmTArray <SmCurve *> &rCurves) 
{
    for (ULONG i = 0; i < rCurves.GetSize(); i++) 
    {
        if (rCurves[i])
        UserCurves.Add(rCurves[i]);
    }

} // end UserTest::Draw(SmTArray <SmCurve *>)

/**************************************************************
PURPOSE ---

USAGE NOTES ---
**************************************************************/
void UserTest::Draw(SmTArray <SmBSplineCurve *> &rCurves) 
{
  for (ULONG i = 0; i < rCurves.GetSize(); i++) 
    {
      if (rCurves[i])
          UserCurves.Add(rCurves[i]);
    }

} // end UserTest::Draw(SmTArray <SmBSplineCurve *>)

/**************************************************************
PURPOSE ---

USAGE NOTES ---
**************************************************************/
void UserTest::Draw(SmExtent3d  *box)
{
    SmPoint3d mmm = box->GetMin();
    SmPoint3d MMM = box->GetMax();
    SmPoint3d Mmm(MMM.x,mmm.y,mmm.z);
    SmPoint3d mMm(mmm.x,MMM.y,mmm.z);
    SmPoint3d mmM(mmm.x,mmm.y,MMM.z);
    SmPoint3d mMM(mmm.x,MMM.y,MMM.z);
    SmPoint3d MmM(MMM.x,mmm.y,MMM.z);
    SmPoint3d MMm(MMM.x,MMM.y,mmm.z);   
    Draw(&mmm,&Mmm);
    Draw(&mmm,&mMm);
    Draw(&mmm,&mmM);
    Draw(&MMM,&MMm);
    Draw(&MMM,&MmM);
    Draw(&MMM,&mMM);
    Draw(&mMm,&mMM);
    Draw(&mmM,&mMM);
    Draw(&mMm,&MMm);
    Draw(&Mmm,&MmM);
    Draw(&Mmm,&MMm);
    Draw(&mmM,&MmM);
}  // end UserTest::Draw(SmExtent3d)
#endif

#if 0
/**************************************************************
PURPOSE ---

USAGE NOTES ---
**************************************************************/
void UserTest::N_DumpPnt(NL_POINT *Pnt)
{    
    SmVector3d Pt (Pnt->x, Pnt->y, Pnt->z);
    Pt.Dump();
    return;

} // end UserTest::N_DumpPnt

/**************************************************************
PURPOSE ---

USAGE NOTES ---
**************************************************************/
void UserTest::N_DumpPnts(int nPts, NL_POINT *Pnts)
{    
    TCHAR sBuff[SM_TBLOCK_SIZE];
    SM_SPRINTF(sBuff,_T(" %ld\n "),nPts);
    smos_WriteBuffer(sBuff);
    for (int i=0; i< nPts; i++)
    {
        NL_POINT *P = &Pnts[i];
        SmVector3d Pt (P->x, P->y, P->z);
        Pt.Dump();
    }
    return;

} // end UserTest::N_DumpPnt


/**************************************************************
PURPOSE ---

USAGE NOTES ---
**************************************************************/
void UserTest::N_DumpSrf(NL_SURFACE *srf)
{
    if (srf && srf->net == NULL)
        return;
    SmBSplineSurface *pSrf = new(*s_pContext) SmBSplineSurface(srf, FALSE); // FALSE = don't run AssertValidAndHeal on New BSplineSurface
    pSrf->Dump();  
    return;

} // end UserTest::N_DumpSrf
  
/**************************************************************
PURPOSE ---

USAGE NOTES ---
**************************************************************/
void UserTest::Print(TCHAR * str)
{    
    TCHAR sBuff[SM_TBLOCK_SIZE];
    SM_SPRINTF(sBuff,_T("%s"), str);
    smos_WriteBuffer(sBuff);
    
    return;

} // end UserTest::Print

/**************************************************************
PURPOSE ---

USAGE NOTES ---
**************************************************************/
void UserTest::Print(TCHAR * str, ULONG I)
{    
    UserTest::Print(str, (int) I);    
    return;

} // end UserTest::Print

/**************************************************************
PURPOSE ---

USAGE NOTES ---
**************************************************************/
void UserTest::Print(TCHAR * str, int I)
{    
    TCHAR sBuff[SM_TBLOCK_SIZE];
    TCHAR str2[SM_TBLOCK_SIZE];
    smos_WStrCpy(str2, SM_TBLOCK_SIZE, str);
    smos_WStrCat(str2, _T(" %d\n"));
    SM_SPRINTF(sBuff,_T("%s"), str2, I);
    smos_WriteBuffer(sBuff);
    
    return;

} // end UserTest::Print

/**************************************************************
PURPOSE ---

USAGE NOTES ---
**************************************************************/
void UserTest::Print(TCHAR * str, double A)
{    
    TCHAR sBuff[SM_TBLOCK_SIZE];
    TCHAR str2[SM_TBLOCK_SIZE];
    smos_WStrCpy(str2, SM_TBLOCK_SIZE, str);
    smos_WStrCat(str2, _T(" %16.16lf\n"));
    SM_SPRINTF(sBuff,_T("%s"), str2, A);
    smos_WriteBuffer(sBuff);
    
    return;

} // end UserTest::Print

/**************************************************************
PURPOSE ---

USAGE NOTES ---
**************************************************************/
void UserTest::N_DrawLine(NL_POINT *P0, NL_POINT *P1)

{      
    SmPoint3d Pnt0(P0->x, P0->y, P0->z);
    SmPoint3d Pnt1(P1->x, P1->y, P1->z);
    Draw(&Pnt0, &Pnt1);           
    return;

} // end UserTest::N_DrawLine

#endif
